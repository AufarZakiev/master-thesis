#include <string>
#include <cmath>
#include "../include/headers/Variables.h"

Variables::Variables()
{
  storage.insert(std::pair("los_clearance_distance", std::nullopt));       // d_l
  storage.insert(std::pair("los_clearance_care_distance", std::nullopt));  // d_lr
  storage.insert(std::pair("small_positive_constant", 0.2f));
  storage.insert(std::pair("robots_avoidance_distance", std::nullopt));     // d_c
  storage.insert(std::pair("desired_distance", std::nullopt));              // d_r
  storage.insert(std::pair("neighbourhood_distance", std::nullopt));        // d_n
  storage.insert(std::pair("obstacle_care_distance", std::nullopt));        // d_or
  storage.insert(std::pair("obstacles_avoidance_distance", std::nullopt));  // d_o
  storage.insert(std::pair("edge_deletion_distance", std::nullopt));        // d_del
  storage.insert(std::pair("sensing_distance", std::nullopt));              // d_s
  // distance on that potential
  // function of obstacle avoidance
  // starts to increase
  storage.insert(std::pair("derivative_epsilon", 0.01f));
  storage.insert(std::pair("equality_case", 0.00001f));  // for checking numbers for equality
  storage.insert(std::pair("k1", std::nullopt));         // design parameters in collision potential function
  storage.insert(std::pair("k2", std::nullopt));         // design parameters in collision potential function
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

bool Variables::setParam(const std::string& param_name, double value)
{
  if (storage.count(param_name) != 1)
    return false;
  storage[param_name] = value;
  return true;
}

bool Variables::getParam(const std::string& param_name, double& value_ref) const
{
  if (storage.count(param_name) != 1)
    return false;
  value_ref = storage.find(param_name)->second.value();
  return true;
};

ValidatedVariables::ValidatedVariables(Variables v)
{
  double ROBOTS_AVOIDANCE_DISTANCE, NEIGHBOURHOOD_DISTANCE, SENSING_DISTANCE, OBSTACLES_AVOIDANCE_DISTANCE,
      EDGE_DELETION_DISTANCE, DESIRED_DISTANCE, OBSTACLE_CARE_DISTANCE;
  v.getParam("robots_avoidance_distance", ROBOTS_AVOIDANCE_DISTANCE);
  v.getParam("neighbourhood_distance", NEIGHBOURHOOD_DISTANCE);
  v.getParam("sensing_distance", SENSING_DISTANCE);
  v.getParam("obstacles_avoidance_distance", OBSTACLES_AVOIDANCE_DISTANCE);
  v.getParam("obstacle_care_distance", OBSTACLE_CARE_DISTANCE);
  v.getParam("edge_deletion_distance", EDGE_DELETION_DISTANCE);
  v.getParam("desired_distance", DESIRED_DISTANCE);

  bool assumptionOfDistances =
      ROBOTS_AVOIDANCE_DISTANCE < NEIGHBOURHOOD_DISTANCE && NEIGHBOURHOOD_DISTANCE < SENSING_DISTANCE;
  bool preservationGuarantee = sqrt(OBSTACLES_AVOIDANCE_DISTANCE * OBSTACLES_AVOIDANCE_DISTANCE +
                                    NEIGHBOURHOOD_DISTANCE * NEIGHBOURHOOD_DISTANCE) <= SENSING_DISTANCE;
  bool deactivationCase = EDGE_DELETION_DISTANCE < ROBOTS_AVOIDANCE_DISTANCE * sin(M_PI / 3.0);
  bool robotAvoidanceDistances =
      ROBOTS_AVOIDANCE_DISTANCE < DESIRED_DISTANCE && DESIRED_DISTANCE < NEIGHBOURHOOD_DISTANCE;
  bool obstacleAvoidanceDistances = OBSTACLES_AVOIDANCE_DISTANCE < OBSTACLE_CARE_DISTANCE;
  if (!(assumptionOfDistances && preservationGuarantee && deactivationCase && robotAvoidanceDistances &&
        obstacleAvoidanceDistances))
  {
    throw std::invalid_argument(
        "Invalid Variables object. assumptionOfDistances: " + std::to_string(assumptionOfDistances) +
        "\npreservationGuarantee: " + std::to_string(preservationGuarantee) + "\ndeactivationCase: " +
        std::to_string(deactivationCase) + "\nrobotAvoidanceDistances: " + std::to_string(robotAvoidanceDistances) +
        "\nobstacleAvoidanceDistances: " + std::to_string(obstacleAvoidanceDistances));
  }
}

bool ValidatedVariables::getParam(const std::string& param_name, double& value_ref) const
{
  if (storage.count(param_name) != 1)
    return false;
  value_ref = storage.find(param_name)->second;
  return true;
}
