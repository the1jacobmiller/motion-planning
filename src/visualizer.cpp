#include "motion_planning/visualizer.h"

void Visualizer::publishTrajectory(vector<point> &trajectory_pts) {
  visualization_msgs::Marker trajectory_marker;
  trajectory_marker.header.frame_id = "/world";
  trajectory_marker.header.stamp = ros::Time::now();
  trajectory_marker.ns = "points_and_lines";
  trajectory_marker.action = visualization_msgs::Marker::ADD;
  trajectory_marker.pose.orientation.w = 1.0;
  trajectory_marker.id = 0;
  trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_marker.scale.x = 1.0;
  trajectory_marker.scale.y = 1.0;
  trajectory_marker.scale.z = 1.0;
  trajectory_marker.color.r = 0;
  trajectory_marker.color.g = 1;
  trajectory_marker.color.b = 0;
  trajectory_marker.color.a = 1.0;

  for (int i = 0; i < trajectory_pts.size(); i++) {
    geometry_msgs::Point p;
    p.x = trajectory_pts[i].x;
    p.y = trajectory_pts[i].y;
    p.z = 1.0;
    trajectory_marker.points.push_back(p);
  }

  trajectory_pub.publish(trajectory_marker);
}

void constructTreeMarker(Tree &tree, visualization_msgs::MarkerArray *tree_array, int &id) {
  for (auto branch : tree.children) {
    visualization_msgs::Marker branch_marker;
    branch_marker.header.frame_id = "/world";
    branch_marker.header.stamp = ros::Time::now();
    branch_marker.ns = "points_and_lines";
    branch_marker.action = visualization_msgs::Marker::ADD;
    branch_marker.pose.orientation.w = 1.0;
    branch_marker.id = id++;
    branch_marker.type = visualization_msgs::Marker::LINE_STRIP;
    branch_marker.scale.x = 0.75;
    branch_marker.scale.y = 0.75;
    branch_marker.scale.z = 0.75;
    branch_marker.color.r = 1;
    branch_marker.color.g = 1;
    branch_marker.color.b = 0;
    branch_marker.color.a = 1.0;

    geometry_msgs::Point p1;
    p1.x = tree.data.position.x;
    p1.y = tree.data.position.y;
    p1.z = 0;
    branch_marker.points.push_back(p1);

    geometry_msgs::Point p2;
    p2.x = branch->data.position.x;
    p2.y = branch->data.position.y;
    p2.z = 0;
    branch_marker.points.push_back(p2);

    // ROS_INFO_STREAM("Added marker - x: " << branch->data.position.x << ", y: " << branch->data.position.y);
    // ROS_INFO_STREAM("             - x: " << tree.data.position.x << ", y: " << tree.data.position.y);
    tree_array->markers.push_back(branch_marker);
    constructTreeMarker(*branch, tree_array, id);
  }
}

void Visualizer::publishTree(Tree &tree) {
  visualization_msgs::MarkerArray tree_array;
  visualization_msgs::Marker del_marker;
  del_marker.header.frame_id = "/world";
  del_marker.action = visualization_msgs::Marker::DELETEALL;
  tree_array.markers.push_back(del_marker);
  tree_pub.publish(tree_array);
  tree_array.markers[0].action = visualization_msgs::Marker::ADD;

  int id = 0;
  constructTreeMarker(tree, &tree_array, id);
  // ROS_ERROR_STREAM("Tree has " << tree.children.size() << " children");
  ROS_ERROR_STREAM("Publishing " << tree_array.markers.size() << " markers");
  tree_pub.publish(tree_array);
}

void Visualizer::publishGoal(point goal) {
  visualization_msgs::Marker goal_marker;
  goal_marker.header.frame_id = "/world";
  goal_marker.header.stamp = ros::Time::now();
  goal_marker.ns = "points_and_lines";
  goal_marker.action = visualization_msgs::Marker::ADD;
  goal_marker.pose.orientation.w = 1.0;
  goal_marker.id = 0;
  goal_marker.type = visualization_msgs::Marker::POINTS;
  goal_marker.scale.x = 1.5;
  goal_marker.scale.y = 1.5;
  goal_marker.scale.z = 1.5;
  goal_marker.color.r = 1;
  goal_marker.color.g = 0;
  goal_marker.color.b = 0;
  goal_marker.color.a = 1.0;

  geometry_msgs::Point p1;
  p1.x = goal.x;
  p1.y = goal.y;
  p1.z = 2.0;
  goal_marker.points.push_back(p1);
  goal_pub.publish(goal_marker);
}

void Visualizer::publish_object_markers(autoware_msgs::DetectedObjectArray msg, vehicle_state ford_state) {
  visualization_msgs::MarkerArray allObjects;
  int it = 0;

  visualization_msgs::Marker del_marker;
  del_marker.header.frame_id = "/world";
  del_marker.action = visualization_msgs::Marker::DELETEALL;
  allObjects.markers.push_back(del_marker);
  object_markers_pub.publish(allObjects);
  allObjects.markers[0].action = visualization_msgs::Marker::ADD;

  for (auto object : msg.objects) {
    visualization_msgs::Marker object_marker;
    object_marker.header.frame_id = "/world";
    object_marker.header.stamp = ros::Time::now();
    object_marker.ns = "objects";
    object_marker.action = visualization_msgs::Marker::ADD;
    // object_marker.lifetime = ros::Duration(0.1);
    object_marker.pose.orientation.w = 1.0;
    object_marker.id = it++;
    object_marker.type = visualization_msgs::Marker::CUBE;

    object_marker.color.r = 0.0;
    object_marker.color.g = 0.0;
    object_marker.color.b = 1.0;
    object_marker.color.a = 1.0;

    // define scale
    geometry_msgs::Vector3 v;
    v.x = object.dimensions.x;
    v.y = object.dimensions.y;
    v.z = object.dimensions.z;
    object_marker.scale = v;

    // translate object to localization position and orientation
    geometry_msgs::Point p;
    p.x = (object.pose.position.x * cos(-ford_state.yaw) - object.pose.position.y * sin(-ford_state.yaw)) + ford_state.position.x;
    p.y = (object.pose.position.x * sin(-ford_state.yaw) + object.pose.position.y * cos(-ford_state.yaw)) + ford_state.position.y;
    p.z = 0.0;
    geometry_msgs::Pose pos;
    pos.position = p;
    object_marker.pose = pos;

    allObjects.markers.push_back(object_marker);
  }

  object_markers_pub.publish(allObjects);
}
