#include <ros/init.h>
#include "headers/dakai_algo.h"

void getNotifiedParam(ros::NodeHandle& n_, const std::string& param_name, double& param_variable)
{
  if (n_.getParam(param_name, param_variable))
  {
    ROS_INFO("Got param %s: %f", param_name.c_str(), param_variable);
  }
  else
  {
    ROS_ERROR("Failed to get param %s. Setting to default value", param_name.c_str());
  }
}

void getNotifiedParam(ros::NodeHandle& n_, const std::string& param_name, int& param_variable)
{
  if (n_.getParam(param_name, param_variable))
  {
    ROS_INFO("Got param %s: %d", param_name.c_str(), param_variable);
  }
  else
  {
    ROS_ERROR("Failed to get param %s. Setting to default value", param_name.c_str());
  }
}

double getVectorDistance(const Vector_t& v1, const Vector_t& v2)
{
  double x_coord = v1(0, 0) - v2(0, 0);
  double y_coord = v1(1, 0) - v2(1, 0);
  return sqrt(x_coord * x_coord + y_coord * y_coord);
}

RigidObject::RigidObject(Position_t position)
{
  current_position_ = position;
}

Position_t RigidObject::getPosition() const
{
  return current_position_;
}

double Robot::getUmax() const
{
  return u_max_;
}

Position_t getRelativePosition(const RigidObject& o1, const RigidObject& o2)
{
  // get position of o2 in respect to o1
  return o2.getPosition() - o1.getPosition();
};
double getVectorLength(const Vector_t& v)
{
  return sqrt(v(0, 0) * v(0, 0) + v(1, 0) * v(1, 0));
};
double getVectorLength(const Eigen::Vector3d& v)
{
  return sqrt(v(0, 0) * v(0, 0) + v(1, 0) * v(1, 0) + v(2, 0) * v(2, 0));
};
bool isEdgePreserved(const Robot& i, const Robot& j)
{
  // indicator function prototype
  return true;
};
bool isObjectOnLineSegment(const RigidObject& o, const RigidObject& line_start, const RigidObject& line_end)
{
  // check if the object o is on the line between objects
  Position_t o_s = getRelativePosition(o, line_start);
  Position_t o_e = getRelativePosition(o, line_end);
  Eigen::Vector3d o_s_3d(o_s(0, 0), o_s(1, 0), 0);
  Eigen::Vector3d o_e_3d(o_e(0, 0), o_e(1, 0), 0);
  bool isPointOnLine = (getVectorLength(o_s_3d.cross(o_e_3d)) == 0.0);
  bool isPointBetweenSegmentEnds = (o_s.dot(o_e) <= 0.0);
  return isPointOnLine && isPointBetweenSegmentEnds;
};
bool isObjectInDSpace(const RigidObject& o, const RigidObject& left_border, const RigidObject& right_border)
{
  // check if the object is in the D-space
  Position_t o_l = getRelativePosition(left_border, o);
  Position_t o_r = getRelativePosition(right_border, o);
  Position_t r_l = getRelativePosition(left_border, right_border);
  return (o_l.dot(r_l) > 0.0 && o_r.dot(r_l) < 0.0);
};
double getProjectionPhi(const Vector_t& p, const Vector_t& q)
{
  // get projection of vector p on the line orthogonal to q
  return true;
};
bool isObjectInTSpace(const RigidObject& m, const RigidObject& i, const RigidObject& j)
{
  // check if (i,j,m) forms T set
  return true;
};