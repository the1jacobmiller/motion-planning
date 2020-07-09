#ifndef __MOTION_PLANNER_H__
#define __MOTION_PLANNER_H__

#include "motion_planning/point.h"
#include "motion_planning/vehicle_state.h"
#include "motion_planning/util.h"
#include "motion_planning/visualizer.h"
#include "motion_planning/occupancy_distribution.h"

#include <ros/ros.h>
#include <fstream>

#include <std_msgs/Float32.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <geographic_msgs/GeographicMap.h>
#include <dbw_mkz_msgs/SteeringReport.h>

using namespace std;

ros::Timer update_timer;
ros::Publisher speed_pub;

Visualizer vis;
Occupancy_Distribution occupancy_dist;

vector<point> target_path;
vehicle_state ford_state;

// Global Parameters
float control_period;

#endif
