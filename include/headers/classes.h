#ifndef PROJECT_DAKAI_ALGO_H
#define PROJECT_DAKAI_ALGO_H

#include <ros/ros.h>
#include <ros/init.h>

#include "Variables.h"
#include "../Eigen/Dense"
#include "../gnuplot-iostream/gnuplot-iostream.h"

#include <cmath>
#include <algorithm>
#include <tuple>
#include <utility>
#include <boost/graph/adjacency_list.hpp>  // graph implementation
#include <unordered_set>                   // fast set implementation

typedef Eigen::Vector2d Position_t;  // to store objects' (robots, obstacles) positions in respect to (0,0) point
typedef Eigen::Vector2d Vector_t;    // to store vectors between points

class RigidObject
{
public:
  RigidObject();
  explicit RigidObject(Position_t position);
  Position_t getPosition() const;
  void setPosition(Position_t position);

protected:
  Position_t current_position_;  // current Object position
};

class Robot : public RigidObject
{
public:
  Robot() = default;
  explicit Robot(Position_t position, Vector_t current_speed_direction = Vector_t(0, 0),
                 double max_speed_value = std::numeric_limits<double>::max());

  const Vector_t getSpeedDirection() const;
  void setSpeedDirection(const Vector_t& current_speed_direction);

  double getMaxSpeedValue() const;
  void setMaxSpeedValue(double max_speed_value);

  double getSpeedValue() const;

  int getRobotID() const;

  ~Robot();

private:
  static int robots_count;
  int ID;
  Vector_t current_speed_direction_;
  double max_speed_value_;
};

class Obstacle : public RigidObject
{
public:
  Obstacle() = default;
  explicit Obstacle(Position_t position, double radius = 0);

  double getRadius() const;
  void setRadius(double radius);

  int getObstacleID() const;

  ~Obstacle();

private:
  double radius_;
  static int obstacles_count;
  int ID;
};

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Robot> RobotGraph;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Obstacle> ObstacleGraph;

typedef boost::graph_traits<RobotGraph>::vertex_descriptor RobotDesc;
typedef boost::graph_traits<ObstacleGraph>::vertex_descriptor ObstacleDesc;

#endif  // PROJECT_DAKAI_ALGO_H
