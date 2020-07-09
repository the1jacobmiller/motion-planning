#ifndef __MOTION_PREDICTION_H__
#define __MOTION_PREDICTION_H__

#include "motion_planning/util.h"
#include "motion_planning/vehicle_state.h"

#include <autoware_msgs/DetectedObjectArray.h>

using namespace std;

autoware_msgs::DetectedObjectArray predictObstacleMotion(autoware_msgs::DetectedObjectArray obstacles, vehicle_state ford_state, float t);

#endif
