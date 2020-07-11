#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include "motion_planning/point.h"
#include "motion_planning/tree.h"
#include "motion_planning/vehicle_state.h"
#include "motion_planning/occupancy_distribution.h"
#include "motion_planning/visualizer.h"

using namespace std;

ros::Publisher trajectory_pub, speed_pub;
ros::Timer rrt_timer;

vehicle_state ford_state;
Occupancy_Distribution occ_dist;
Visualizer vis;

double dt = 2.0;
double target_threshold;
point goal_position;

float k_d, k_yaw, uniqueness_threshold, max_steering_angle, min_steering_angle, max_time;

void read_configs() {
  if (!ros::param::get("/motion_planning/target_threshold", target_threshold) ||
  !ros::param::get("/motion_planning/max_steering_angle", max_steering_angle) ||
  !ros::param::get("/motion_planning/max_time", max_time) ||
  !ros::param::get("/motion_planning/k_d", k_d) ||
  !ros::param::get("/motion_planning/k_yaw", k_yaw) ||
  !ros::param::get("/motion_planning/uniqueness_threshold", uniqueness_threshold) ||
  !ros::param::get("/motion_planning/goal_x", goal_position.x) ||
  !ros::param::get("/motion_planning/goal_y", goal_position.y)) {
    ROS_ERROR_STREAM("Failed to read ROS configs from rrt");
    ros::shutdown();
  }
  min_steering_angle = -1.0 * max_steering_angle;
}

void toEulerAngle(const geometry_msgs::Quaternion &q, double &yaw, double &pitch, double &roll) {
  // roll (x-axis rotation)
  double sinr = +2.0 * (q.w * q.x + q.y * q.z);
  double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  roll = atan2(sinr, cosr);
  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w * q.y - q.z * q.x);
  if (fabs(sinp) >= 1) pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else pitch = asin(sinp);
  // yaw (z-axis rotation)
  double siny = +2.0 * (q.w * q.z + q.x * q.y);
  double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  yaw = atan2(siny, cosy);
}

double getRandom(double max, double min) {
  return (max - min) * ((double)rand() / (double)RAND_MAX) + min;
}

int getRandomInt(int max, int min) {
  return (rand() % max) + min;
}

Tree *getRandomSample(Tree *rrt, double &random_steering_angle) {
  random_steering_angle = getRandom(max_steering_angle, min_steering_angle);

  Tree *tree = rrt;
  int num_children = tree->children.size();
  while(num_children > 0) {
    int rand_child = getRandomInt(num_children, -1);
    if (rand_child == -1) break; // return current branch
    else tree = tree->children[rand_child];
    num_children = tree->children.size();
  }
  return tree;
}

double distance2D(point pt1, point pt2) {
  return sqrt(pow(pt1.x-pt2.x, 2) + pow(pt1.y-pt2.y, 2));
}

Tree *findClosestNode(Tree &rrt, node new_node) {
  Tree *closest_node = &rrt;

  double dist_to_root = distance2D(rrt.data.position, new_node.position);
  double yaw_diff = fabs(rrt.data.yaw - new_node.yaw);
  double min_distance = k_d*dist_to_root + k_yaw*yaw_diff;
  for (auto branch : rrt.children) {
    Tree *closest_child = findClosestNode(*branch, new_node);

    double dist_to_child = distance2D(closest_child->data.position, new_node.position);
    double yaw_diff = fabs(closest_child->data.yaw - new_node.yaw);
    double distance = k_d*dist_to_child + k_yaw*yaw_diff;
    if (distance < min_distance) {
      min_distance = distance;
      closest_node = closest_child;
    }
  }

  return closest_node;
}

bool checkGoal(node leaf, point goal) {
  double distance = distance2D(leaf.position, goal);
  if (distance < target_threshold) return true;
  return false;
}

node extrapolateNode(node closest_node, double steering_angle) {
  double vehicle_speed_ms = max(2.0, ford_state.speed_ms);

  node last_node = closest_node;
  double t = 0.0;
  while (t < max_time) {
    double yaw_rate = vehicle_speed_ms * tan(steering_angle) / wheelbase;
    node ext_node;
    ext_node.position.x = closest_node.position.x + vehicle_speed_ms*t * cos(closest_node.yaw + yaw_rate*t);
    ext_node.position.y = closest_node.position.y + vehicle_speed_ms*t * sin(closest_node.yaw + yaw_rate*t);
    ext_node.yaw = closest_node.yaw + yaw_rate*t;

    // ROS_WARN_STREAM("t: " << t << ", yaw_rate: " << yaw_rate << ", x:" << ext_node.position.x << ", y:" << ext_node.position.y);

    float probability;
    if (!occ_dist.isFeasible(ext_node.position, ford_state, probability)) {
      return last_node;
    }
    if (checkGoal(ext_node, goal_position)) return ext_node;
    last_node = ext_node;
    t += 0.25;
  }

  return last_node;
}

vector<point> findPath(Tree &rrt, vector<point> &path) {
  if (rrt.isGoal) {
    path.push_back(rrt.data.position);
    return path;
  }
  else if (rrt.children.size() == 0) {
    // is leaf but is not goal
    path.clear();
    return path;
  }
  for (auto branch : rrt.children) {
    vector<point> local_path = path;
    local_path.push_back(branch->data.position);
    vector<point> new_path = findPath(*branch, local_path);
    if (new_path.size() > 0) return new_path;
  }
  path.clear();
  return path;
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

void localizationCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  ford_state.position.x = msg->pose.position.x;
  ford_state.position.y = msg->pose.position.y;
  double roll, pitch;
  toEulerAngle(msg->pose.orientation, ford_state.yaw, pitch, roll);
  ford_state.localized = true;

  // check if arrived at goal
  bool reached_goal = distance2D(ford_state.position, goal_position) < 2.0;
  if (reached_goal) {
    std_msgs::Float32 speed_msg;
    speed_msg.data = 0.0;
    speed_pub.publish(speed_msg);
  }

}

void obstacleCallback(autoware_msgs::DetectedObjectArray msg) {
  occ_dist.obstacles_detected = true;
  occ_dist.obstacles = msg;

  vis.publish_object_markers(msg, ford_state);
}

void rrt_callback(const ros::TimerEvent& event) {
  static bool foundPath = false;

  if (foundPath == true) return;

  float speed;
  if(!ros::param::get("/motion_planning/speed", speed)) {
    ROS_ERROR_STREAM("Failed to read ros_configs from rrt_callback");
    ros::shutdown();
  }

  if (!ford_state.localized || !occ_dist.obstacles_detected) return;

  ROS_INFO_STREAM("Begin RRT");
  Tree rrt(ford_state.position, ford_state.yaw);

  int iter = 1;
  bool done = checkGoal(rrt.data, goal_position);
  while(!done) {
    double random_steering_angle;
    Tree *rand_node = getRandomSample(&rrt, random_steering_angle);
    // ROS_INFO_STREAM("Got new random sample");
    node ext_node = extrapolateNode(rand_node->data, random_steering_angle);
    // ROS_INFO_STREAM("Extrapolated node");

    Tree *closest_node = findClosestNode(rrt, ext_node);
    double dist_to_node = distance2D(closest_node->data.position, ext_node.position);
    double yaw_diff = fabs(closest_node->data.yaw - ext_node.yaw);
    double distance = k_d*dist_to_node + k_yaw*yaw_diff;
    if (distance < uniqueness_threshold) continue;

    Tree *new_leaf = new Tree;
    new_leaf->data = ext_node;

    // check if new node can connect to goal
    done = checkGoal(ext_node, goal_position);
    if (done) {
      // add goal leaf to tree
      Tree *goal_leaf = new Tree;
      goal_leaf->data.position = goal_position;
      goal_leaf->isGoal = true;
      new_leaf->children.emplace_back(goal_leaf);
    }

    // add edge to tree
    rand_node->children.push_back(new_leaf);
    iter++;

    vis.publishTree(rrt);
    vis.publishGoal(goal_position);

    std_msgs::Float32 speed_msg;
    speed_msg.data = speed;
    speed_pub.publish(speed_msg);
  }
  // vis.publishTree(rrt);
  // vis.publishGoal(goal_position);
  ROS_WARN_STREAM("Done building tree");

  // publish path to goal to control
  vector<point> path;
  path.push_back(rrt.data.position);
  path = findPath(rrt, path);
  path = interpolate_points(path);
  path = interpolate_points(path);
  ROS_ERROR_STREAM("Done finding path");

  if (path.size() > 0) {
    vis.publishTrajectory(path);
    foundPath = true;
  }

  ROS_ERROR_STREAM("Done with rrt_callback!");
}

int main(int argc, char **argv) {
  // initialize ros node
  ros::init(argc, argv, "rrt_node");
  ros::NodeHandle n;
  ros::Rate r(30);

  read_configs();

  // Publishers
  vis.trajectory_pub = n.advertise<visualization_msgs::Marker>("/motion_planning/trajectory", 1); // publishes to MPC
  vis.object_markers_pub = n.advertise<visualization_msgs::MarkerArray>("/obstacle_markers", 1);
  vis.tree_pub = n.advertise<visualization_msgs::MarkerArray>("/tree", 1);
  vis.goal_pub = n.advertise<visualization_msgs::Marker>("/goal", 1);
  speed_pub = n.advertise<std_msgs::Float32>("/motion_planning/speed", 1); // publishes to MPC

  // Subscribers
  ros::Subscriber sub_localization = n.subscribe("/gnss_pose", 1, localizationCallback);
  ros::Subscriber sub_obstacles = n.subscribe("/detection/object_tracker/objects", 1, obstacleCallback);
  // ros::Subscriber sub_steering_report = n.subscribe("/vehicle/steering_report", 1, recvSteeringReport);
  // ros::Subscriber sub_accel = n.subscribe("/vehicle/filtered_accel", 1, recvAccel);

  rrt_timer = n.createTimer(ros::Duration(dt), rrt_callback);

  ros::MultiThreadedSpinner spinner(0); // use one thread for each CPU core
  spinner.spin();

}
