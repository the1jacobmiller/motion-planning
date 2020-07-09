#include "motion_planning/occupancy_distribution.h"

using namespace std;

float Occupancy_Distribution::calcOccupancyProbability(point p, float t, vehicle_state ford_state) {
  double max_uncertainty_multiplier;
  if (!ros::param::get("/motion_planning/max_uncertainty_multiplier", max_uncertainty_multiplier)) {
      ROS_ERROR_STREAM("Failed to read ROS configs from calcOccupancyProbability");
      ros::shutdown();
  }

  if (!obstacles_detected) return 0.0;

  // predict obstacle motion
  autoware_msgs::DetectedObjectArray predicted_obstacles = predictObstacleMotion(obstacles, ford_state, t);

  // sum the probability densities for each obstacle
  double total_probability = 0.0;
  for (int i = 0; i < predicted_obstacles.objects.size(); i++) {
    autoware_msgs::DetectedObject obstacle = predicted_obstacles.objects[i];

    // translate obstacle into vehicle's frame of reference
    long double mu_x = (obstacle.pose.position.x * cos(ford_state.yaw) - obstacle.pose.position.y * sin(ford_state.yaw)) + ford_state.position.x;
    long double mu_y = (obstacle.pose.position.x * sin(ford_state.yaw) + obstacle.pose.position.y * cos(ford_state.yaw)) + ford_state.position.y;

    // ROS_WARN_STREAM("mu_x - ford_state.position.x: " << mu_x - ford_state.position.x);
    // ROS_WARN_STREAM("mu_y - ford_state.position.y: " << mu_y - ford_state.position.y);

    // assign variances based on the obstacle score (or variance)
    // as score lowers, sigma increases
    // TODO - check if variance is being set for obstacles
    float sigma_x = min(1.0/obstacle.score, max_uncertainty_multiplier);
    float sigma_y = min(1.0/obstacle.score, max_uncertainty_multiplier);

    // sigma_x = 5.0;
    // sigma_y = 5.0;

    obstacle.dimensions.x += vehicle_length;
    obstacle.dimensions.y += vehicle_width;

    long double x_max = p.x + obstacle.dimensions.x/2.0;
    long double x_min = p.x - obstacle.dimensions.x/2.0;

    long double y_max = p.y + obstacle.dimensions.y/2.0;
    long double y_min = p.y - obstacle.dimensions.y/2.0;

    // ROS_WARN_STREAM("mu_x - x_max: " << mu_x - x_max);
    // ROS_WARN_STREAM("mu_y - y_max: " << mu_y - y_max);

    // Calculate probability based on Gaussian Distribution
    float probability = fabs(cdf(mu_x, sigma_x, x_max) - cdf(mu_x, sigma_x, x_min));
    probability *= fabs(cdf(mu_y, sigma_y, y_max) - cdf(mu_y, sigma_y, y_min));

    total_probability += probability;
  }
  // ROS_WARN_STREAM("Probability: " << total_probability);
  return total_probability;
}

bool Occupancy_Distribution::isFeasible(point node, float t, vehicle_state ford_state, float &probability) {
  float occupancy_threshold;
  if (!ros::param::get("/motion_planning/occupancy_threshold", occupancy_threshold)) {
      ROS_ERROR_STREAM("Failed to read ROS configs from isFeasible");
      ros::shutdown();
  }

  probability = calcOccupancyProbability(node, t, ford_state);
  if (probability > occupancy_threshold) {
    // ROS_ERROR_STREAM("Node is not feasible");
    return false;
  }
  else return true;
}
