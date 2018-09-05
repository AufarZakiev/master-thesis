#ifndef PROJECT_DAKAI_ALGO_H
#define PROJECT_DAKAI_ALGO_H

#include <ros/ros.h>
#include "../Eigen/Dense"
#include "Variables.h"
#include "../templates/DakaiAlgo.tcc"
#include "RobotGraph.h"
#include "RigidObject.h"
#include <cmath>

typedef Eigen::Vector2d Position_t;  // to store objects' (robots, obstacles) positions in respect to (0,0) point
typedef Eigen::Vector2d Vector_t;    // to store vectors between points

typedef Eigen::Vector2d ControlInput_t;   // to store control input vectors
typedef std::set<Obstacle> Obstacle_set;  // to store info about math sets
typedef std::set<Robot> Robot_set;        // to store info about math sets

void getNotifiedParam(ros::NodeHandle& n_, const std::string& param_name, Variables& v);
double getVectorDistance(const Vector_t& v1, const Vector_t& v2);  // get distance between two vectors
double get2DVectorLength(const Vector_t& v);                       // get 2d vector length
double getVector3DLength(const Eigen::Vector3d& v);                // get 3d vector length
double getSquaredVectorLength(const Vector_t& v);  // get 2d vector squared length to save precision when it used
                                                   // squared
Position_t getRelativePosition(const RigidObject& o1, const RigidObject& o2);  // get position of o2 in respect to o1
bool isEdgePreserved(const Robot& i, const Robot& j, const RobotGraph& rg,
                     const Variables& v);  // indicator function prototype
double angleBetweenVectorsInRadians(const Vector_t& v1,
                                    const Vector_t& v2);  // get the angle between two vectors in counter-clockwise
                                                          // direction in radians
bool isObjectOnLineSegment(const RigidObject& o, const RigidObject& line_start,
                           const RigidObject& line_end);  // check if the object o is on the line between objects
bool isObjectInDSpace(const RigidObject& o, const RigidObject& left_border,
                      const RigidObject& right_border);  // check if the object is in the D-space
Vector_t getProjectionPhi(const Vector_t& p,
                          const Vector_t& q);  // get projection of vector p on the line orthogonal to q
bool isObjectInTSet(const Robot& i, const Robot& j, const Robot& m, const RobotGraph& rg,
                    const Variables& v);  // check if (i,j,m) forms T set
bool isObjectInDashedTSet(const Robot& i, const Robot& j, const Robot& m, const RobotGraph& rg, const Variables& v);  // check if (i,j,m) forms T-dash set
double phiInterrobotCollisionPotential(double z, const Variables& v);

double psiInterrobotCollisionPotential(const Robot& i, const RobotGraph& robot_graph,
                                       const Variables& v);  // psi potential function depending on interrobot distance

double phiObstacleCollisionPotential(double z,
                                     const Variables& v);  // phi potential function depending on distance to obstacles

double phiLOSPreservePotential(double z, const Variables& v);  // potential function depending on LOS preservation

double phiCohesionPotential(double z, const Variables& v);  // potential function of group cohesion

double psiObstacleCollisionPotential(const Robot& i, const Obstacle_set& current_obstacle_set, const Variables& v);

double psiLOSPreservePotential(const Robot& i, const Obstacle_set& i_obstacle_set,
                               const Robot_set& i_neighbourhood_sigma_set,
                               const Obstacle_set& current_obstacle_set_in_D, const Variables& v);

double psiCohesionPotential(const Robot& i, const Robot_set& sensed_robots_set, const Variables& v);

double potentialFunction(const Robot_set& all_robots, const RobotGraph& robot_graph, const Obstacle_set& i_obstacle_set,
                         const Variables& v, const Robot_set& neighbourhood_robots_sgima,
                         const Obstacle_set& i_obstacle_set_in_D, const Robot_set& sensed_robots_set);

#endif  // PROJECT_DAKAI_ALGO_H
