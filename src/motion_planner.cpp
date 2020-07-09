#include "motion_planning/motion_planner.h"

using namespace std;

double determineSpeed(vector<point> target_path) {
  int prior_pt_index, forward_pt_index;
  if(!find2ClosestPoints(target_path, ford_state.position, prior_pt_index, forward_pt_index)) {
    return 0.0;
  }
  double speed = target_path[forward_pt_index].speed;

  return speed;
}

/* ----------------------------------- Callbacks --------------------------------------- */

int findKey(geographic_msgs::WayPoint waypoint, string key) {
  for (int i = 0; i < waypoint.props.size(); i++) {
    if (key.compare(waypoint.props[i].key) == 0) return i;
  }
  return -1;
}

double getSpeedFromWaypoint(geographic_msgs::WayPoint waypoint) {
  string key = "speed";
  int index = findKey(waypoint, key);
  string::size_type sz;
  double speed = stod(waypoint.props[index].value, &sz);
  return speed;
}

void pathCallback(const geographic_msgs::GeographicMap::ConstPtr &msg) {
  if (msg->points.size() == 0) {
    return;
  }

  vector<point> new_target_path;
  for (auto pt : msg->points) {
    double speed = getSpeedFromWaypoint(pt);
    point new_point(pt.position.longitude, pt.position.latitude);
    new_point.speed = speed;
    new_target_path.push_back(new_point);
  }
  target_path = new_target_path;
}

void obstacleCallback(autoware_msgs::DetectedObjectArray msg) {
  occupancy_dist.obstacles = msg;
  occupancy_dist.obstacles_detected = true;

  vis.publish_object_markers(msg, ford_state);
}

void localizationCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  double localizer_pitch, localizer_roll;
  geometry_msgs::Pose pose = msg->pose;
  toEulerAngle(pose.orientation, ford_state.yaw, localizer_pitch, localizer_roll);
  ford_state.position = point(pose.position.x, pose.position.y, ford_state.yaw);
  ford_state.localized = true;
}

void recvSteeringReport(const dbw_mkz_msgs::SteeringReport::ConstPtr& msg) {
	ford_state.speed_ms = msg->speed;
  ford_state.steering_wheel_angle = msg->steering_wheel_angle;
}

/* ----------------------------------- Control --------------------------------------- */

bool step(float t, point &start_position, float k_prob_) {
  static float prev_yaw_rate;

  float yaw_rate_threshold, min_speed, distance_limit, k_d, k_p, k_a;
  int num_random_samples;
  if (!ros::param::get("/motion_planning/yaw_rate_threshold", yaw_rate_threshold) ||
  !ros::param::get("/motion_planning/num_random_samples", num_random_samples) ||
  !ros::param::get("/motion_planning/min_speed", min_speed) ||
  !ros::param::get("/motion_planning/distance_limit", distance_limit) ||
  !ros::param::get("/motion_planning/k_d", k_d) ||
  !ros::param::get("/motion_planning/k_p", k_p) ||
  !ros::param::get("/motion_planning/k_a", k_a)) {
    ROS_ERROR_STREAM("Motion Planner - Error reading ROS configs in step");
    ros::shutdown();
  }

  double yaw_rate_min = -1 * yaw_rate_threshold;
  double yaw_rate_max = yaw_rate_threshold;

  float vehicle_speed_ms = max((float)ford_state.speed_ms, min_speed);

  float min_cost = numeric_limits<float>::max();
  float best_yaw_rate, end_yaw;
  point end_position;

  bool success = false;
  for (int i = 0; i < num_random_samples; i++) { // iterate through random samples
    float yaw_rate = getRandom(yaw_rate_max, yaw_rate_min); // generate random yaw rate
    // ROS_INFO_STREAM("Motion Planner - Yaw Rate " << yaw_rate);

    // predict vehicle's motion based on the generated yaw rate
    point node, cp1, cp2;
    node.x = start_position.x + vehicle_speed_ms * control_period * cos(start_position.yaw + yaw_rate * control_period);
    node.y = start_position.y + vehicle_speed_ms * control_period * sin(start_position.yaw + yaw_rate * control_period);
    node.yaw = start_position.yaw + yaw_rate * control_period;

    float probability;
    if (!occupancy_dist.isFeasible(node, t, ford_state, probability)) { // check if node is feasible
      continue;
    }

    // calculate distance to the path
    int cp1_index, cp2_index;
    find2ClosestPoints(target_path, node, cp1_index, cp2_index);
    cp1 = target_path[cp1_index];
    cp2 = target_path[cp2_index];
    point projectedP = findClosestProjectedPoint(cp1, cp2, node);
    float d = distance2DPoints(node, projectedP);
    if (d > distance_limit) continue;
    success = true;

    // calculate cost- greedy search
    float p = fabs(prev_yaw_rate - yaw_rate);
    float a = yaw_rate;
    float cost = k_d * d + k_p * p + k_a * a + k_prob_ * probability;
    if (cost < min_cost) {
      min_cost = cost;
      best_yaw_rate = yaw_rate;
      end_position = node;
      end_yaw = start_position.yaw + yaw_rate * control_period;
    }
  }

  if (!success) {
    ROS_ERROR_STREAM("Motion Planner - Did not find feasible node");
    return false;
  }

  start_position = end_position;
  prev_yaw_rate = best_yaw_rate;
  // ROS_INFO_STREAM("Motion Planner - Best Yaw Rate " << best_yaw_rate);
  return true;
}

float check_trajectory_feasibility(vector<point> current_trajectory, vector<point> &trajectory_pts) {
  float max_dist_to_traj;
  if (!ros::param::get("/motion_planning/max_dist_to_traj", max_dist_to_traj)) {
      ROS_ERROR_STREAM("Failed to read ROS configs from check_trajectory_feasibility");
      ros::shutdown();
  }

  int prior_pt_index, forward_pt_index;
  if (!find2ClosestPoints(current_trajectory, ford_state.position, prior_pt_index, forward_pt_index)) {
    trajectory_pts.push_back(ford_state.position);
    return control_period;
  }
  point projectedPoint = findClosestProjectedPoint(current_trajectory[prior_pt_index], current_trajectory[forward_pt_index], ford_state.position);
  if (distance2DPoints(ford_state.position, projectedPoint) > max_dist_to_traj) {
    trajectory_pts.push_back(ford_state.position);
    return control_period;
  }

  for (int i = prior_pt_index; i < current_trajectory.size(); i++) {
    float t = i * control_period;
    float probability;
    if (!occupancy_dist.isFeasible(current_trajectory[i], t, ford_state, probability)) return max(t, control_period);
    trajectory_pts.push_back(current_trajectory[i]);
  }
  return current_trajectory.size() * control_period;
}

vector<point> smooth_trajectory(vector<point> trajectory_pts) {
  float path_smoothing;
  if (!ros::param::get("/motion_planning/path_smoothing", path_smoothing)) {
      ROS_ERROR_STREAM("Failed to read ROS configs from smooth_trajectory");
      ros::shutdown();
  }

  vector<point> final_pts;

  point pt_0 = trajectory_pts[0];
  point pt_f = trajectory_pts[trajectory_pts.size()-1];
  final_pts.push_back(pt_0);
  for (int i = 1; i < trajectory_pts.size()-1; i++) {
    point projected_pt = findClosestProjectedPoint(pt_0, pt_f, trajectory_pts[i]);
    point smoothed_pt;
    smoothed_pt.x = path_smoothing * trajectory_pts[i].x + (1.0 - path_smoothing) * projected_pt.x;
    smoothed_pt.y = path_smoothing * trajectory_pts[i].y + (1.0 - path_smoothing) * projected_pt.y;
    final_pts.push_back(smoothed_pt);
  }
  final_pts.push_back(pt_f);

  return final_pts;
}

vector<point> interpolate_points(vector<point> trajectory_pts) {
  vector<point> final_pts;
  final_pts.push_back(trajectory_pts[0]);
  for (int i = 0; i < trajectory_pts.size()-1; i++) {
    point middle_pt;
    middle_pt.x = (trajectory_pts[i].x + trajectory_pts[i+1].x)/2.0;
    middle_pt.y = (trajectory_pts[i].y + trajectory_pts[i+1].y)/2.0;
    final_pts.push_back(middle_pt);
    final_pts.push_back(trajectory_pts[i+1]);
  }

  return final_pts;
}

void update(const ros::TimerEvent& event) {
  static vector<point> current_trajectory;

  float max_time_step, k_prob;
  if (!ros::param::get("/motion_planning/max_time_step", max_time_step) ||
  !ros::param::get("/motion_planning/k_prob", k_prob)) {
      ROS_ERROR_STREAM("Failed to read ROS configs from update");
      ros::shutdown();
  }

  if (target_path.size() == 0) return;

  vector<point> trajectory_pts;
  ROS_WARN_STREAM("Checking trajectory feasibility");
  float begin_time = check_trajectory_feasibility(current_trajectory, trajectory_pts);
  float k_prob_ = k_prob;
  bool success = false;
  while (!success && begin_time < max_time_step) {
    for (float t = begin_time; t < max_time_step; t += control_period) { // for each time step
      ROS_WARN_STREAM("Motion Planner - Step " << t);

      point position;
      if (trajectory_pts.size() > 0) position = trajectory_pts.back();
      else position = ford_state.position;
      // calculate a feasible steering commands
      success = step(t, position, k_prob_);

      if (!success) {
        // publish empty trajetory and try again to find safe trajectory
        ROS_ERROR_STREAM("Motion Planner - Failed to find safe trajectory");
        begin_time = 0.0;
        trajectory_pts.clear();
        trajectory_pts.push_back(ford_state.position);
        vis.publishTrajectory(trajectory_pts);
        k_prob_ *= 1.1;
        break;
      }
      else trajectory_pts.push_back(position);
    }

  }
  ROS_INFO_STREAM("Motion Planner - Finished finding safe trajectory");

  vector<point> smoothed_pts = smooth_trajectory(trajectory_pts);
  // vector<point> final_pts = interpolate_points(smoothed_pts);
  vis.publishTrajectory(smoothed_pts);
  current_trajectory = trajectory_pts;

  double speed = determineSpeed(target_path);
  std_msgs::Float32 speed_msg;
  speed_msg.data = speed;
  speed_pub.publish(speed_msg);
}

int main(int argc, char **argv) {
  // initialize ros node
  ros::init(argc, argv, "motion_planner_node");
  ros::NodeHandle n;
  ros::Rate r(30);

  if (!ros::param::get("/motion_planning/control_period", control_period)) {
    ROS_ERROR_STREAM("Motion Planner - Failed to read ROS configs in main");
    ros::shutdown();
  }

  // Publishers
  vis.trajectory_pub = n.advertise<visualization_msgs::Marker>("/motion_planning/trajectory", 1); // publishes to control
  speed_pub = n.advertise<std_msgs::Float32>("/motion_planning/speed", 1); // publishes to control
  vis.object_markers_pub = n.advertise<visualization_msgs::MarkerArray>("/obstacles", 1);

  // Subscribers
  ros::Subscriber sub_localization = n.subscribe("/gnss_pose", 1, localizationCallback);;
  ros::Subscriber sub_path = n.subscribe("/path_planning/path", 1, pathCallback);
  ros::Subscriber sub_obstacles = n.subscribe("/detection/object_tracker/objects", 1, obstacleCallback);
  ros::Subscriber sub_steering_report = n.subscribe("/vehicle/steering_report", 1, recvSteeringReport);

  update_timer = n.createTimer(ros::Duration(control_period), update);

  ros::MultiThreadedSpinner spinner(0); // use one thread for each CPU core
  spinner.spin();

}
