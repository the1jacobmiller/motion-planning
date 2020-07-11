#ifndef POINT_H
#define POINT_H

struct point {
    double x;
    double y;
    double yaw;

    point() {}
    point(double x_, double y_) {
        x=x_;
        y=y_;
    }
};

struct node {
  point position;
  double yaw;

  node () {}
  node (point p, double angle) {
    position = p;
    yaw = angle;
  }
};

#endif
