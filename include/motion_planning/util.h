#ifndef __UTIL_H__
#define __UTIL_H__

#include <ros/ros.h>
#include <string>
#include <vector>

#include <geometry_msgs/Quaternion.h>

#include "motion_planning/point.h"

using namespace std;

void toEulerAngle(const geometry_msgs::Quaternion &q, double &yaw, double &pitch, double &roll);
geometry_msgs::Quaternion toQuaternion(double pitch, double roll, double yaw);
double distance2DPoints(point point1, point point2);
double getAngleToPoint(point point1, point point2);
double getAngleDiff(double angle1, double angle2);
double ms2mph(double speed_ms);
double mph2ms(double speed_mph);
bool find2ClosestPoints(vector<point> points, point position, int &prior_pt_index, int &forward_pt_index);
point findClosestProjectedPoint(point cp1, point cp2, point target_point);
double getRandom(double max, double min);
float cdf(float mu, float sigma, float value);

#endif
