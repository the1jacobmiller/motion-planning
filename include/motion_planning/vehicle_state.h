#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H

#include "motion_planning/point.h"

#define wheelbase 2.8498; // 112.2 inches
#define vehicle_width 1.8542
#define vehicle_length 4.8768
#define steering_ratio 14.8

struct vehicle_state {
    point position;
    double yaw;
    double speed_ms;
    double steering_wheel_angle;
    bool localized;

    vehicle_state() {}
};

#endif
