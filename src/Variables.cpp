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
  storage.insert(std::pair<std::string, double>("small_positive_constant", 0.1f));
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