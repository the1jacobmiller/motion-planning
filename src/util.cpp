#include "motion_planning/util.h"

using namespace std;

void toEulerAngle(const geometry_msgs::Quaternion &q, double &yaw, double &pitch, double &roll) {
  // roll (x-axis rotation)
  double sinr = +2.0 * (q.w * q.x + q.y * q.z);
  double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  roll = atan2(sinr, cosr);
  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w * q.y - q.z * q.x);
  if (fabs(sinp) >= 1) pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else pitch = asin(sinp);
  // yaw (z-axis rotation)
  double siny = +2.0 * (q.w * q.z + q.x * q.y);
  double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  yaw = atan2(siny, cosy);
}

geometry_msgs::Quaternion toQuaternion(double pitch, double roll, double yaw) {
  geometry_msgs::Quaternion q;
  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  q.w = cy * cr * cp + sy * sr * sp;
  q.x = cy * sr * cp - sy * cr * sp;
  q.y = cy * cr * sp + sy * sr * cp;
  q.z = sy * cr * cp - cy * sr * sp;
  return q;
}

double distance2DPoints(point point1, point point2) {
  return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
}

double getAngleToPoint(point point1, point point2) {
  double diff_lon = point1.x - point2.x;
  double diff_lat = point1.y - point2.y;

  if (diff_lon == 0.0) diff_lon = 0.000001;
  double angle = atan(diff_lat / diff_lon);

  if (diff_lat >= 0 && diff_lon >= 0); // Q1
  else if (diff_lat >= 0 && diff_lon <= 0) { // Q2
    angle = M_PI + angle;
  } else if (diff_lat <= 0 && diff_lon <= 0) { // Q3
    angle = -1 * M_PI + angle;
  } else if (diff_lat <= 0 && diff_lon >= 0); // Q4

  return angle;
}

double getAngleDiff(double angle1, double angle2) {
  double diff = angle1 - angle2;

  if (diff > M_PI) diff += -2 * M_PI;
  if (diff < -M_PI) diff += 2 * M_PI;

  return diff;
}

double ms2mph(double speed_ms) {
  return speed_ms * 2.23694;
}

double mph2ms(double speed_mph) {
  return speed_mph * 0.447;
}

bool find2ClosestPoints(vector<point> points, point position, int &prior_pt_index, int &forward_pt_index) {
  if (points.size() == 0) {
    ROS_ERROR_STREAM("Motion Planner - No path data");
    return false;
  }

  int begin_check, max_check;
  begin_check = 0;
  max_check = points.size() - 1;

  int closestIndex;
  double minimumDist = numeric_limits<double>::max();
  for (int i = begin_check; i < max_check; i++) {
    double dist = distance2DPoints(points[i], position);
    if (dist < minimumDist) { minimumDist = dist; closestIndex = i; }
  }

  if (closestIndex == 0) { prior_pt_index = 0; forward_pt_index = 1; }
  else if (closestIndex == points.size()-1) { prior_pt_index = closestIndex-1; forward_pt_index = closestIndex; }
  else {
    double angle_closest = getAngleToPoint(points[closestIndex], position);

    double angle_prior_diff = getAngleDiff(getAngleToPoint(points[closestIndex-1], position), angle_closest);
    double angle_forward_diff = getAngleDiff(getAngleToPoint(points[closestIndex+1], position), angle_closest);

    if (fabs(angle_forward_diff) < fabs(angle_prior_diff)) {
      prior_pt_index = closestIndex-1;
      forward_pt_index = closestIndex;
    }
    else {
      prior_pt_index = closestIndex;
      forward_pt_index = closestIndex+1;
    }
  }

  return true;
}

point findClosestProjectedPoint(point cp1, point cp2, point target_point) {
  // return a virtual point, aka a point on the axis of cp1 and cp2, that is projected
  // lon1 & lat1 - longitude and latitude of closest point
  // lon2 & lat2 - longitude and latitude of second closest point
  // lon3 & lat3 - longitude and latitude of current position
  point projectedPoint;

  long double m = (cp2.y - cp1.y) / (cp2.x - cp1.x);
  if (cp2.x - cp1.x == 0) {
    m = 1000000;
  }
  long double b = cp1.y - m * cp1.x;
  long double m_perp = -1 * (cp2.x - cp1.x) / (cp2.y - cp1.y);
  if (cp2.y - cp1.y == 0) {
    // if m_perp is inf
    m_perp = -1000000;
  }
  long double b_perp = target_point.y - m_perp * target_point.x;
  projectedPoint.x = (b_perp - b) / (m - m_perp);
  projectedPoint.y = m * projectedPoint.x + b;

  return projectedPoint;
}

double getRandom(double max, double min) {
  return (max - min) * ((double)rand() / (double)RAND_MAX) + min;
}

float cdf(float mu, float sigma, float value) {
  // returns the CDF of a Gaussian distribution
  // return (1.0 + erf((value-mu)/(sigma*sqrt(2))))/2.0;
  return 0.5 * erfc(-(value-mu)/(sigma*sqrt(2)));
}
