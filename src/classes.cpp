#include "../include/headers/classes.h"
#include "../include/headers/geometric_functions.h"

int Robot::robots_count = 0;
int Obstacle::obstacles_count = 0;

RigidObject::RigidObject()
{
  current_position_ << 0, 0;
}

RigidObject::RigidObject(Position_t position)
{
  current_position_ = std::move(position);
}

Position_t RigidObject::getPosition() const
{
  return current_position_;
}

void RigidObject::setPosition(Position_t position)
{
  current_position_ = std::move(position);
}

Robot::Robot(Position_t position, Vector_t current_speed_direction, double max_speed_value)
  : RigidObject(std::move(position))
{
  this->current_speed_direction_ = std::move(current_speed_direction);
  this->ID = robots_count;
  robots_count++;
}

double Robot::getSpeedValue() const
{
  return getVectorLength(current_speed_direction_);
}

const Vector_t Robot::getSpeedDirection() const
{
  return current_speed_direction_;
}

void Robot::setSpeedDirection(const Vector_t& current_speed_direction)
{
  this->current_speed_direction_ = current_speed_direction;
}

int Robot::getRobotID() const
{
  return ID;
}

void Robot::updatePosition()
{
  this->current_position_ += this->current_speed_direction_;
}

Robot::~Robot()
{
  robots_count--;
}

Obstacle::Obstacle(Position_t position, double radius)
{
  this->radius_ = radius;
  this->current_position_ = std::move(position);
  this->ID = obstacles_count;
  obstacles_count++;
}

double Obstacle::getRadius() const
{
  return radius_;
}

void Obstacle::setRadius(double radius)
{
  this->radius_ = radius;
}

int Obstacle::getObstacleID() const
{
  return ID;
}

Obstacle::~Obstacle()
{
  obstacles_count--;
}

ValidatedGraphs::ValidatedGraphs(std::unique_ptr<RobotGraph> rg, std::unique_ptr<ObstacleGraph> og, const ValidatedVariables& vv)
{
  double OBSTACLES_AVOIDANCE_DISTANCE, ROBOTS_AVOIDANCE_DISTANCE;
  vv.getParam("obstacles_avoidance_distance", OBSTACLES_AVOIDANCE_DISTANCE);
  vv.getParam("robots_avoidance_distance", ROBOTS_AVOIDANCE_DISTANCE);

  for (size_t robot_id = 0; robot_id < boost::num_vertices(*rg); robot_id++)
  {
    for (size_t obstacle_id = 0; obstacle_id < boost::num_vertices(*og); obstacle_id++)
    {
      if (getVectorLength(getRelativePosition((*rg)[robot_id], (*og)[obstacle_id])) <
          OBSTACLES_AVOIDANCE_DISTANCE + (*og)[obstacle_id].getRadius())
      {
        throw std::invalid_argument("Robot ID " + std::to_string((*rg)[robot_id].getRobotID()) +
                                    " is too close to Obstacle ID " +
                                    std::to_string((*og)[obstacle_id].getObstacleID()));
      }
    }
  }

  for (size_t robot_id = 0; robot_id < boost::num_vertices(*rg); robot_id++)
  {
    for (size_t robot_id2 = 0; robot_id2 < boost::num_vertices(*rg); robot_id2++)
    {
      if (robot_id != robot_id2 &&
          getVectorLength(getRelativePosition((*rg)[robot_id], (*rg)[robot_id2])) < ROBOTS_AVOIDANCE_DISTANCE)
      {
        throw std::invalid_argument("Robot ID " + std::to_string((*rg)[robot_id].getRobotID()) +
                                    " is too close to Robot ID" + std::to_string((*rg)[robot_id2].getRobotID()));
      }
    }
  }


  validatedObstacleGraph = std::move(og);
  validatedRobotGraph = std::move(rg);

  this->leavePreservedEdges(vv);

  std::vector<int> component(boost::num_vertices(*validatedRobotGraph));
  if (boost::connected_components(*validatedRobotGraph, &component[0]) != 1)
  {
    throw std::invalid_argument("Robot graph is not connected.");
  }
}

RobotGraph& ValidatedGraphs::getRobotGraph()
{
  return *validatedRobotGraph;
}

ObstacleGraph& ValidatedGraphs::getObstacleGraph()
{
  return *validatedObstacleGraph;
}

void ValidatedGraphs::leavePreservedEdges(const ValidatedVariables& vv)
{
  double NEIGHBOURHOOD_DISTANCE;
  vv.getParam("neighbourhood_distance", NEIGHBOURHOOD_DISTANCE);
  auto& robots = *validatedRobotGraph;
  for (size_t it = 0; it != boost::num_vertices(robots); it++)
  {
    for (size_t it2 = 0; it2 != boost::num_vertices(robots); it2++)
    {
      if (it != it2)
      {
        auto robot = robots[it];
        auto robotTarget = robots[it2];
        if (getVectorLength(getRelativePosition(robot, robotTarget)) < NEIGHBOURHOOD_DISTANCE &&
            isEdgePreserved(robot, robotTarget, robots, Variables(vv)))
        {
          boost::add_edge(it, it2, robots);
        }
        else
        {
          boost::remove_edge(it, it2, robots);
        }
      }
    }
  }
}