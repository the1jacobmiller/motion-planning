#ifndef __OCCUPANCY_DISTRIBUTION_H__
#define __OCCUPANCY_DISTRIBUTION_H__

#include <ros/ros.h>
#include "motion_planning/point.h"
#include "motion_planning/vehicle_state.h"

#include <autoware_msgs/DetectedObjectArray.h>

using namespace std;

class Occupancy_Distribution {
  public:
    autoware_msgs::DetectedObjectArray obstacles;
    bool obstacles_detected;

    bool isFeasible(point node, vehicle_state ford_state, float &probability);

  private:
    float calcOccupancyProbability(point p, vehicle_state ford_state);
};

#endif
