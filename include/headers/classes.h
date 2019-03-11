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
  Robot() = default;  // TODO: why does make_pair complain about it??
  explicit Robot(Position_t position, Vector_t current_speed_direction = Vector_t(0, 0), double current_speed_value = 0,
                 double max_speed_value = std::numeric_limits<double>::max());
  double getSpeedValue() const;
  void setSpeedValue(double current_speed_value);

  const Vector_t getSpeedDirection() const;
  void setSpeedDirection(const Vector_t& current_speed_direction);

  double getMaxSpeedValue() const;
  void setMaxSpeedValue(double max_speed_value);
  
  int getID() const;

private:
  static int robots_count;
  int ID;
  double current_speed_value_;        // current speed of movement
  Vector_t current_speed_direction_;  // current speed of direction
  double max_speed_value_;
};

class Obstacle : public RigidObject
{
public:
  Obstacle() = default;
  explicit Obstacle(Position_t position);

  double getRadius() const;
  void setRadius(double radius);
private:
  double radius_;
};

struct Edge
{
};
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Robot, Edge> RobotGraph;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Obstacle, Edge> ObstacleGraph;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, RigidObject, Edge> RigidGraph;  // to store
                                                                                                          // information
                                                                                                          // about
                                                                                                          // connections
typedef boost::graph_traits<RobotGraph>::vertex_descriptor RobotDesc;
typedef Eigen::Vector2d ControlInput_t;         // to store control input vectors
typedef std::unordered_set<RigidObject> Set_t;  // to store info about math sets

#endif  // PROJECT_DAKAI_ALGO_H
