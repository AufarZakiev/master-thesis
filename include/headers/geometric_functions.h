#ifndef MULTIPLE_TURTLEBOTS_GEOMETRIC_FUNCTIONS_H
#define MULTIPLE_TURTLEBOTS_GEOMETRIC_FUNCTIONS_H

#include "classes.h"
#include "../templates/derivatives.tcc"

std::optional<RobotDesc> findRobotInGraph(const Robot& ro, const RobotGraph& graph);
std::optional<ObstacleDesc> findObstacleInGraph(const Obstacle& ro, const ObstacleGraph& graph);

double getVectorDistance(const Vector_t& v1, const Vector_t& v2);  // get distance between two vectors
double getVectorLength(const Vector_t& v);                         // get 2d vector length
double getVectorLength(const Eigen::Vector3d& v);                  // get 3d vector length
double getSquaredVectorLength(const Vector_t& v);  // get 2d vector squared length to save precision when it used
// squared
Position_t getRelativePosition(const RigidObject& o1, const RigidObject& o2);  // get position of o2 in respect to o1
bool isEdgePreserved(const Robot& i, const Robot& j, const RobotGraph& robots,
                     const Variables& v);  // indicator function
double angleBetweenVectorsInRadians(const Vector_t& v1,
                                    const Vector_t& v2);  // get the angle between two vectors in counter-clockwise
// direction in radians
bool isObjectOnLineSegment(const RigidObject& o, const RigidObject& line_start, const RigidObject& line_end,
                           const Variables& v);  // check if the object o is on the line between objects
bool isObjectInDSpace(const RigidObject& o, const RigidObject& left_border,
                      const RigidObject& right_border);  // check if the object is in the D-space
Vector_t getProjectionPhi(const Vector_t& p,
                          const Vector_t& q);  // get projection of vector p on the line orthogonal to q
bool isEdgeInGraph(const Robot& i, const Robot& j,
                   const RobotGraph& rg);  // Check if the edge with vertices i,j exists in graph rg
bool isObjectInTSet(const Robot& i, const Robot& j, const Robot& m, const RobotGraph& rg,
                    const Variables& v);  // check if (i,j,m) forms T set
bool isObjectInDashedTSet(const Robot& i, const Robot& j, const Robot& m, const RobotGraph& rg,
                          const Variables& v);  // check if (i,j,m) forms T-dash set

#endif  // MULTIPLE_TURTLEBOTS_GEOMETRIC_FUNCTIONS_H
