#include <string>
#include "../include/headers/Variables.h"

Variables::Variables()
{
  storage.insert(std::pair<std::string, double>("robots_count", -1.0f));
  storage.insert(std::pair<std::string, double>("robots_avoidance_distance", -1.0f));
  storage.insert(std::pair<std::string, double>("obstacles_avoidance_distance", -1.0f));
  storage.insert(std::pair<std::string, double>("sensing_distance", -1.0f));
  storage.insert(std::pair<std::string, double>("los_clearance_distance", -1.0f));
  storage.insert(std::pair<std::string, double>("neighbourhood_distance", -1.0f));
  storage.insert(std::pair<std::string, double>("edge_deletion_distance", -1.0f));
  storage.insert(
      std::pair<std::string, double>("small_positive_constant", 0.1f));  // for checking distances for equality
  storage.insert(std::pair<std::string, double>("derivative_epsilon", 0.01f));
  storage.insert(std::pair<std::string, double>("equality_case", 0.00001f));  // for checking numbers for equality
  storage.insert(std::pair<std::string, double>("desired_distance", -1.0f));    // desired distance between robots
  storage.insert(std::pair<std::string, double>("k1", 10.0f));  // design parameters in collision potential function
  storage.insert(std::pair<std::string, double>("k2", 10.0f));  // design parameters in collision potential function
}

Variables& Variables::getInstance()
{
  static Variables instance;
  return instance;
}

bool Variables::setParam(std::string param_name, double value)
{
  if (storage.count(param_name) != 1)
    return false;
  storage[param_name] = value;
  return true;
}

bool Variables::getParam(std::string param_name, double& value_ref) const
{
  if (storage.count(param_name) != 1)
    return false;
  value_ref = storage.find(param_name)->second;
  return true;
};