#ifndef PROJECT_DAKAI_ALGO_H
#define PROJECT_DAKAI_ALGO_H

#include <ros/ros.h>
#include "../Eigen/Dense"
#include "Variables.h"
#include "../templates/DakaiAlgo.tcc"
#include <cmath>
#include <boost/graph/adjacency_list.hpp>  // graph implementation
#include <unordered_set>                   // fast set implementation
#include <algorithm>

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
  explicit Robot(Position_t position);
  double getUmax() const;

private:
  double u_max_;  // maximum speed of movement
};

class Obstacle : public RigidObject
{
public:
  explicit Obstacle(Position_t position);
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
typedef boost::graph_traits<RigidGraph>::vertex_descriptor RigidObjectDesc;
typedef boost::graph_traits<RigidGraph>::edge_descriptor EdgeDesc;
typedef boost::graph_traits<RigidGraph>::adjacency_iterator adjacency_it;
typedef Eigen::Vector2d ControlInput_t;         // to store control input vectors
typedef std::unordered_set<RigidObject> Set_t;  // to store info about math sets

#endif  // PROJECT_DAKAI_ALGO_H
