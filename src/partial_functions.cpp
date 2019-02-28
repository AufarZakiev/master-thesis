#include "../include/headers/partial_functions.h"

double partialInterrobotCollisionPotential(double z, const Variables& v)
{
  double ROBOTS_AVOIDANCE_DISTANCE, NEIGHBOURHOOD_DISTANCE, DESIRED_DISTANCE, K1, K2;
  v.getParam("robots_avoidance_distance", ROBOTS_AVOIDANCE_DISTANCE);
  v.getParam("neighbourhood_distance", NEIGHBOURHOOD_DISTANCE);
  v.getParam("desired_distance", DESIRED_DISTANCE);
  v.getParam("k1", K1);
  v.getParam("k2", K2);
  if (z > NEIGHBOURHOOD_DISTANCE || z < ROBOTS_AVOIDANCE_DISTANCE)
    return 0;
  double part1 = [&](double z) {
    return (z - DESIRED_DISTANCE) * (z - DESIRED_DISTANCE) * (NEIGHBOURHOOD_DISTANCE - z) /
           ((NEIGHBOURHOOD_DISTANCE - ROBOTS_AVOIDANCE_DISTANCE) *
            (NEIGHBOURHOOD_DISTANCE - ROBOTS_AVOIDANCE_DISTANCE) * (z - ROBOTS_AVOIDANCE_DISTANCE) +
            (DESIRED_DISTANCE - ROBOTS_AVOIDANCE_DISTANCE) * (DESIRED_DISTANCE - ROBOTS_AVOIDANCE_DISTANCE) *
            (NEIGHBOURHOOD_DISTANCE - z) / K1);
  }(z);
  double part2 = [&](double z) {
    return (z - DESIRED_DISTANCE) * (z - DESIRED_DISTANCE) * (z - ROBOTS_AVOIDANCE_DISTANCE) /
           ((NEIGHBOURHOOD_DISTANCE - ROBOTS_AVOIDANCE_DISTANCE) *
            (NEIGHBOURHOOD_DISTANCE - ROBOTS_AVOIDANCE_DISTANCE) * (NEIGHBOURHOOD_DISTANCE - z) +
            (ROBOTS_AVOIDANCE_DISTANCE - DESIRED_DISTANCE) * (ROBOTS_AVOIDANCE_DISTANCE - DESIRED_DISTANCE) *
            (z - ROBOTS_AVOIDANCE_DISTANCE) / K2);
  }(z);
  return part1 + part2;
}

double partialObstacleCollisionPotential(double z, const Variables& v)
{
  double OBSTACLE_CARE_DISTANCE, OBSTACLE_AVOIDANCE_DISTANCE, SMALL_POSITIVE_CONSTANT;
  v.getParam("obstacle_care_distance", OBSTACLE_CARE_DISTANCE);
  v.getParam("obstacles_avoidance_distance", OBSTACLE_AVOIDANCE_DISTANCE);
  v.getParam("small_positive_constant", SMALL_POSITIVE_CONSTANT);
  double potential = 0;
  if (z >= OBSTACLE_CARE_DISTANCE || z < OBSTACLE_AVOIDANCE_DISTANCE)
    return 0;
  potential = (1.0 / ((z - OBSTACLE_AVOIDANCE_DISTANCE) / (OBSTACLE_CARE_DISTANCE - OBSTACLE_AVOIDANCE_DISTANCE) +
                      SMALL_POSITIVE_CONSTANT)) -
              (1.0 / (1.0 + SMALL_POSITIVE_CONSTANT));
  potential = potential * potential / 2;
  return potential;
}

double partialLOSPreservePotential(double z, const Variables& v)
{
  double LOS_CLEARANCE_DISTANCE, LOS_CLEARANCE_CARE_DISTANCE, SMALL_POSITIVE_CONSTANT;
  v.getParam("los_clearance_distance", LOS_CLEARANCE_DISTANCE);
  v.getParam("los_clearance_care_distance", LOS_CLEARANCE_CARE_DISTANCE);
  v.getParam("small_positive_constant", SMALL_POSITIVE_CONSTANT);
  double potential = 0;
  if (z >= LOS_CLEARANCE_CARE_DISTANCE || z < LOS_CLEARANCE_DISTANCE)
    return 0;
  potential = (1.0 / ((z - LOS_CLEARANCE_DISTANCE) / (LOS_CLEARANCE_CARE_DISTANCE - LOS_CLEARANCE_DISTANCE) +
                      SMALL_POSITIVE_CONSTANT)) -
              (1.0 / (1.0 + SMALL_POSITIVE_CONSTANT));
  potential = potential * potential / 2;
  return potential;
}

double partialCohesionPotential(double z, const Variables& v)
{
  double NEIGHBOURHOOD_DISTANCE;
  v.getParam("neighbourhood_distance", NEIGHBOURHOOD_DISTANCE);
  double potential = 0;
  if (z <= NEIGHBOURHOOD_DISTANCE)
    return 0;
  potential = (z - NEIGHBOURHOOD_DISTANCE) * (z - NEIGHBOURHOOD_DISTANCE) / 2;  // TODO: must be no obstacles around
  return potential;
}
