#ifndef POINT_H
#define POINT_H

struct point {
    double x;
    double y;
    double yaw;
    double speed;

    point() {}
    point(double x_, double y_) {
        x=x_;
        y=y_;
    }
    point(double x_, double y_, double yaw_) {
        x=x_;
        y=y_;
        yaw=yaw_;
    }
    point(double x_, double y_, double yaw_, double speed_) {
        x=x_;
        y=y_;
        yaw=yaw_;
        speed=speed_;
    }
};

#endif
