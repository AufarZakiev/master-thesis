#ifndef PROJECT_DAKAI_ALGO_H
#define PROJECT_DAKAI_ALGO_H

#include <ros/init.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <cmath>

void get_param(ros::NodeHandle& n_, const std::string& param_name, double& param_variable);
void get_param(ros::NodeHandle& n_, const std::string& param_name, int& param_variable);
double vector_distance(Eigen::Vector2d& v1, Eigen::Vector2d& v2);

#endif  // PROJECT_DAKAI_ALGO_H
