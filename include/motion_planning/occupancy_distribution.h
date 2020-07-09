#ifndef __OCCUPANCY_DISTRIBUTION_H__
#define __OCCUPANCY_DISTRIBUTION_H__

#include "motion_planning/point.h"
#include "motion_planning/vehicle_state.h"
#include "motion_planning/util.h"
#include "motion_planning/motion_prediction.h"

#include <autoware_msgs/DetectedObjectArray.h>

using namespace std;

class Occupancy_Distribution {
  public:
    autoware_msgs::DetectedObjectArray obstacles;
    bool obstacles_detected;

    bool isFeasible(point node, float t, vehicle_state ford_state, float &probability);

  private:
    float calcOccupancyProbability(point p, float t, vehicle_state ford_state);
};

#endif
