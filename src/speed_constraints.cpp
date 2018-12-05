#include "../include/headers/speed_constraints.h"

double maximumDistanceConstraint(const Robot& robot, const RobotGraph& neighbourhood_preserved_robots,
                                 const Variables& v)
{
  double NEIGHBOURHOOD_DISTANCE;
  v.getParam("neighbourhood_distance", NEIGHBOURHOOD_DISTANCE);
  RobotGraph neighbours_behind, neighbours_front;

  std::pair p = separateNeighbourRobotsBehindAndFront(robot, neighbourhood_preserved_robots);
  neighbours_behind = p.first;
  neighbours_front = p.second;
  auto closestRobotDist = closestRobotDistance(robot, neighbours_behind);
  if (closestRobotDist)
  {
    return 0.5 * (NEIGHBOURHOOD_DISTANCE - closestRobotDist.value());
  }
  else
  {
    return std::numeric_limits<double>::max();  // TODO: good point to avoid exceptions?
  }
};

double maximumDistanceConstraint2(const Robot& robot, const RobotGraph& neighbourhood_preserved_robots,
                                  const Variables& v)
{
  RobotGraph neighbours_behind, neighbours_front;

  std::pair p = separateNeighbourRobotsBehindAndFront(robot, neighbourhood_preserved_robots);
  neighbours_behind = p.first;
  neighbours_front = p.second;
  auto HZ = minimumHZ(robot, neighbours_behind);
  if (HZ)
  {
    return HZ.value();  // TODO: insert fucking direction
  }
  else
  {
    return std::numeric_limits<double>::max();  // TODO: good point to avoid exceptions?
  }
};

double interrobotAvoidanceConstraint(const Robot& robot, const RobotGraph& detected_robots, const Variables& v)
{
  double OBS_AVOIDANCE_DISTANCE;
  v.getParam("obstacles_avoidance_distance", OBS_AVOIDANCE_DISTANCE);

  RobotGraph detected_behind, detected_front;

  std::pair p = separateDetectedRobotsBehindAndFront(robot, detected_robots);
  detected_behind = p.first;
  detected_front = p.second;
  auto closestRobotDist = closestRobotDistance(robot, detected_behind);
  if (closestRobotDist)
  {
    return 0.5 * (closestRobotDist.value() - OBS_AVOIDANCE_DISTANCE);
  }
  else
  {
    return std::numeric_limits<double>::max();  // TODO: good point to avoid exceptions?
  }
};

double obstacleAvoidanceConstraint(const Robot& i, const ObstacleGraph& detected_obstacles, const Variables& v,
                                   double discretization)  // TODO: change for non-circular obstacles
{
  double OBS_AVOIDANCE_DISTANCE;
  v.getParam("obstacles_avoidance_distance", OBS_AVOIDANCE_DISTANCE);

  double min_speed = std::numeric_limits<double>::max();
  for (auto id = 0; id < boost::num_vertices(detected_obstacles); id++)
  {
    double l_c =
        getVectorLength(getProjectionPhi(getRelativePosition(i, detected_obstacles[id]), i.getSpeedDirection()));
    double l_v = sqrt(getRelativePosition(i, detected_obstacles[id]) * getRelativePosition(i, detected_obstacles[id]) -
                      l_c * l_c);

    double speed = l_v - sqrt((OBS_AVOIDANCE_DISTANCE + detected_obstacles[id].getRadius()) *
                                  (OBS_AVOIDANCE_DISTANCE + detected_obstacles[id].getRadius()) -
                              l_c * l_c);
    if (speed < min_speed)
    {
      min_speed = speed;
    }
  }

  return min_speed;
};

double LOSPreservationConstraint(const Robot& robot, const RobotGraph& robots, const ObstacleGraph& detected_obstacles,
                                 const Variables& v)
{
  double LOS_CLEARANCE_DISTANCE;
  v.getParam("los_clearance_distance", LOS_CLEARANCE_DISTANCE);

  auto closest_obstacle = closestObstacleToLOSinDSpaceAtFront(robot, robots[id], detected_obstacles);
  double angle = angleBetweenVectorsInRadeians(robot.getSpeedDirection(),
                                               getRelativePosition(robot.getPosition(), robots[id].getPosition()));
  angle = angle > M_PI ? (angle - M_PI) : angle;
  double speed = (getVectorLength(getProjectionPhi(getRelativePosition(robot, closest_obstacle),
                                                   getRelativePosition(robot, robots[id]))) -
                  LOS_CLEARANCE_DISTANCE) /
                 sin(angle);
}
