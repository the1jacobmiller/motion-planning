#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include "motion_planning/point.h"
#include "motion_planning/vehicle_state.h"
#include "motion_planning/tree.h"

using namespace std;

class Visualizer {
  public:
    // Publishers
    ros::Publisher object_markers_pub;
    ros::Publisher trajectory_pub;
    ros::Publisher tree_pub;
    ros::Publisher goal_pub;

    void publishTrajectory(vector<point> &trajectory_pts);
    void publishTree(Tree &rrt);
    void publishGoal(point goal);
    void publish_object_markers(autoware_msgs::DetectedObjectArray msg, vehicle_state ford_state);

};

#endif
