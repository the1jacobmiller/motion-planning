#include "motion_planning/motion_prediction.h"

using namespace std;

autoware_msgs::DetectedObjectArray predictObstacleMotion(autoware_msgs::DetectedObjectArray obstacles, vehicle_state ford_state, float t) {
  bool predict_motion;
  if (!ros::param::get("/motion_planning/predict_motion", predict_motion)) {
      ROS_ERROR_STREAM("Failed to read ROS configs from predictObstacleMotion");
      ros::shutdown();
  }

  if (!predict_motion) return obstacles;

  autoware_msgs::DetectedObjectArray predicted_obstacles;
  for (int i = 0; i < obstacles.objects.size(); i++) { // for each obstacle
    autoware_msgs::DetectedObject obstacle = obstacles.objects[i];
    // TODO - look into more advanced motion prediction
    double obstacle_speed = sqrt(pow(obstacle.velocity.linear.x,2) + pow(obstacle.velocity.linear.y,2));
    double obstacle_roll, obstacle_pitch, obstacle_yaw;
    toEulerAngle(obstacle.pose.orientation, obstacle_yaw, obstacle_pitch, obstacle_roll);

    // tranform obstacle to global frame
    point obstacle_global;
    obstacle_global.x = cos(-ford_state.yaw)*obstacle.pose.position.x - sin(-ford_state.yaw)*obstacle.pose.position.y + ford_state.position.x;
    obstacle_global.y = sin(-ford_state.yaw)*obstacle.pose.position.x + cos(-ford_state.yaw)*obstacle.pose.position.y + ford_state.position.y;
    double obstacle_yaw_global = obstacle_yaw - ford_state.yaw;

    // propogate motion of obstacle
    point obstacle_predicted_global;
    obstacle_predicted_global.x = obstacle_global.x + obstacle_speed * cos(obstacle_yaw_global) * t;
    obstacle_predicted_global.y = obstacle_global.y + obstacle_speed * sin(obstacle_yaw_global) * t;

    // transform predicted obstacle into new frame
    obstacle.pose.position.x = obstacle_predicted_global.x - ford_state.position.x;
    obstacle.pose.position.y = obstacle_predicted_global.y - ford_state.position.y;
    predicted_obstacles.objects.push_back(obstacle);
  }
  return predicted_obstacles;
}
