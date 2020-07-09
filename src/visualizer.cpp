#include "motion_planning/visualizer.h"

void Visualizer::publishTrajectory(vector<point> trajectory_pts) {
  visualization_msgs::Marker trajectory_marker;
  trajectory_marker.header.frame_id = "/world";
  trajectory_marker.header.stamp = ros::Time::now();
  trajectory_marker.ns = "points_and_lines";
  trajectory_marker.action = visualization_msgs::Marker::ADD;
  trajectory_marker.pose.orientation.w = 1.0;
  trajectory_marker.id = 0;
  trajectory_marker.type = visualization_msgs::Marker::POINTS;
  trajectory_marker.scale.x = 1.0;
  trajectory_marker.scale.y = 1.0;
  trajectory_marker.color.r = 1;
  trajectory_marker.color.g = 1;
  trajectory_marker.color.b = 0;
  trajectory_marker.color.a = 1.0;

  for (int i = 0; i < trajectory_pts.size(); i++) {
    geometry_msgs::Point p;
    p.x = trajectory_pts[i].x;
    p.y = trajectory_pts[i].y;
    p.z = 0;
    trajectory_marker.points.push_back(p);
  }

  trajectory_pub.publish(trajectory_marker);
}

void Visualizer::publish_object_markers(autoware_msgs::DetectedObjectArray msg, vehicle_state ford_state) {
  float max_time_step, control_period;
  if (!ros::param::get("/motion_planning/max_time_step", max_time_step) ||
  !ros::param::get("/motion_planning/control_period", control_period)) {
      ROS_ERROR_STREAM("Failed to read ROS configs from publish_object_markers");
      ros::shutdown();
  }

  visualization_msgs::MarkerArray allObjects;
  int it = 0;

  visualization_msgs::Marker del_marker;
  del_marker.header.frame_id = "/world";
  del_marker.action = visualization_msgs::Marker::DELETEALL;
  allObjects.markers.push_back(del_marker);
  object_markers_pub.publish(allObjects);
  allObjects.markers[0].action = visualization_msgs::Marker::ADD;

  for (float t = 0.0; t < max_time_step; t += control_period) {
    auto predicted_obstacles = predictObstacleMotion(msg, ford_state, t);

    for (auto object : predicted_obstacles.objects) {
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
      object_marker.color.a = 1.0 - t/max_time_step;

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
  }

  object_markers_pub.publish(allObjects);
}
