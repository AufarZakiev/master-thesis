#include "headers/dakai_algo.h"

void get_param(ros::NodeHandle& n_, const std::string& param_name, double& param_variable)
{
  if (n_.getParam(param_name, param_variable))
  {
    ROS_INFO("Got param %s: %d", param_name, param_variable);
  }
  else
  {
    ROS_ERROR("Failed to get param %s. Setting to default value", param_name);
  }
}

void get_param(ros::NodeHandle& n_, const std::string& param_name, int& param_variable)
{
  if (n_.getParam(param_name, param_variable))
  {
    ROS_INFO("Got param %s: %d", param_name, param_variable);
  }
  else
  {
    ROS_ERROR("Failed to get param %s. Setting to default value", param_name);
  }
}

double vector_distance(Eigen::Vector2d& v1, Eigen::Vector2d& v2)
{
  double x_coord = v1(0, 0) - v2(0, 0);
  double y_coord = v1(1, 0) - v2(1, 0);
  return sqrt(x_coord * x_coord + y_coord * y_coord);
}