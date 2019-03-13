#include <string>
#include "../include/headers/Variables.h"

Variables::Variables()
{
  storage.insert(std::pair("los_clearance_distance", std::nullopt));
  storage.insert(std::pair("los_clearance_care_distance", std::nullopt));
  storage.insert(std::pair("small_positive_constant", 0.2f));
  storage.insert(std::pair("robots_avoidance_distance", std::nullopt));
  storage.insert(std::pair("desired_distance", std::nullopt));  // desired distance between robots
  storage.insert(std::pair("neighbourhood_distance", std::nullopt));
  storage.insert(std::pair("obstacle_care_distance", std::nullopt));
  storage.insert(std::pair("obstacles_avoidance_distance", std::nullopt));
  storage.insert(std::pair("edge_deletion_distance", std::nullopt));
    // distance on that potential
  // function of obstacle avoidance
  // starts to increase
  storage.insert(std::pair("derivative_epsilon", 0.001f));
  storage.insert(std::pair("equality_case", 0.00001f));  // for checking numbers for equality
  storage.insert(std::pair("k1", std::nullopt));  // design parameters in collision potential function
  storage.insert(std::pair("k2", std::nullopt));  // design parameters in collision potential function
  storage.insert(std::pair("c1", std::nullopt));  // design weights of potential functions - interrobot collisions
  storage.insert(std::pair("c2", std::nullopt));  // design weights of potential functions - obstacle collisions
  storage.insert(std::pair("c3", std::nullopt));  // design weights of potential functions - LOS preserve
  storage.insert(std::pair("c4", std::nullopt));  // design weights of potential functions - cohesion
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
  value_ref = storage.find(param_name)->second.value();
  return true;
};