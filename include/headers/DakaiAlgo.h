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
#include <optional>

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

std::pair<RigidObjectDesc, bool> findVertexInGraph(const RigidObject& ro, const RigidGraph& graph);
void getNotifiedParam(ros::NodeHandle& n_, const std::string& param_name, Variables& v);
double getVectorDistance(const Vector_t& v1, const Vector_t& v2);  // get distance between two vectors
double getVectorLength(const Vector_t& v);                         // get 2d vector length
double getVectorLength(const Eigen::Vector3d& v);                  // get 3d vector length
double getSquaredVectorLength(const Vector_t& v);  // get 2d vector squared length to save precision when it used
                                                   // squared
Position_t getRelativePosition(const RigidObject& o1, const RigidObject& o2);  // get position of o2 in respect to o1
bool isEdgePreserved(const Robot& i, const Robot& j, const RigidGraph& rg,
                     const Variables& v);  // indicator function
double angleBetweenVectorsInRadians(const Vector_t& v1,
                                    const Vector_t& v2);  // get the angle between two vectors in counter-clockwise
                                                          // direction in radians
bool isObjectOnLineSegment(const RigidObject& o, const RigidObject& line_start,
                           const RigidObject& line_end);  // check if the object o is on the line between objects
bool isObjectInDSpace(const RigidObject& o, const RigidObject& left_border,
                      const RigidObject& right_border);  // check if the object is in the D-space
Vector_t getProjectionPhi(const Vector_t& p,
                          const Vector_t& q);  // get projection of vector p on the line orthogonal to q
bool isVectorInGraph(const RigidObject& i, const RigidObject& j,
                     const RigidGraph& rg);  // Check if the edge with vertices i,j exists in graph rg
bool isObjectInTSet(const RigidObject& i, const RigidObject& j, const RigidObject& m, const RigidGraph& rg,
                    const Variables& v);  // check if (i,j,m) forms T set
bool isObjectInDashedTSet(const RigidObject& i, const RigidObject& j, const RigidObject& m, const RigidGraph& rg,
                          const Variables& v);  // check if (i,j,m) forms T-dash set

std::optional<Obstacle> closestDetectedObstacle(const RigidObject& position, const ObstacleGraph& obstacles_detected,
                                 const Variables& v);

double partialInterrobotCollisionPotential(double z, const Variables &v); // potential function depending on interrobot distance

double interrobotCollisionPotential(const RigidObject& position, const RigidGraph& robots_near_preserved, const Variables& v);

double partialObstacleCollisionPotential(double z, const Variables& v); // potential function depending on distance to obstacles

double obstacleCollisionPotential(const RigidObject &position, const ObstacleGraph &detected_obstcles,
                                  const Variables &v);

std::pair<Obstacle, double> closestObstacleToLOS(const Robot &i, const Robot &j,
                                                 const ObstacleGraph &detected_obstacle_graph_in_D_set); // returns copy of closest Obstacle object and distance to it

Robot j_star_compute(const Robot &i, const RobotGraph &robots_near_preserved,
                     const ObstacleGraph &detected_obstacle_graph_in_D_set);

double partialLOSPreservePotential(double z, const Variables& v); // potential function depending on LOS preservation

double LOSPreservePotential(const Robot& position, const RobotGraph& neighbourhood_robots,
                            const ObstacleGraph& detected_obstacle_graph_in_D_set, const Variables& v);

double partialCohesionPotential(double z, const Variables& v); // potential function of group cohesion

double cohesionPotential(const RigidObject& position, const RigidGraph& all_robots, const Variables& v);


#endif  // PROJECT_DAKAI_ALGO_H
