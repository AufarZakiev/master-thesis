#ifndef PROJECT_DAKAI_ALGO_H
#define PROJECT_DAKAI_ALGO_H

#include <ros/ros.h>
#include "../Eigen/Dense"
#include <cmath>
#include <boost/graph/adjacency_matrix.hpp>  // graph implementation
#include <unordered_set>                     // fast set implementation

typedef Eigen::Vector2d Position_t;  // to store objects' (robots, obstacles) positions in respect to (0,0) point
typedef Eigen::Vector2d Vector_t;    // to store vectors between points

class RigidObject
{
public:
  RigidObject(Position_t position);
  Position_t getPosition() const;

private:
  Position_t current_position_;  // current Object position
};

class Robot : RigidObject
{
public:
  double getUmax() const;

private:
  double u_max_;  // maximum speed of movement
};

class Obstacle : RigidObject
{
};

typedef Eigen::Vector2d ControlInput_t;                        // to store control input vectors
typedef boost::adjacency_matrix<boost::undirectedS> UGraph_t;  // to store information about connections
typedef std::unordered_set<RigidObject> Set_t;                 // to store info about math sets

void getNotifiedParam(ros::NodeHandle& n_, const std::string& param_name, double& param_variable);
void getNotifiedParam(ros::NodeHandle& n_, const std::string& param_name, int& param_variable);
double getVectorDistance(const Vector_t& v1, const Vector_t& v2);  // get distance between two vectors
double getVectorLength(const Vector_t& v);                         // get 2d vector length
double getVectorLength(const Eigen::Vector3d& v);                  // get 3d vector length
double getSquaredVectorLength(const Vector_t& v);  // get 2d vector squared length to save precision when it used
                                                   // squared
Position_t getRelativePosition(const RigidObject& o1, const RigidObject& o2);  // get position of o2 in respect to o1
bool isEdgePreserved(const Robot& i, const Robot& j);                          // indicator function prototype
bool isObjectOnLineSegment(const RigidObject& o, const RigidObject& line_start,
                           const RigidObject& line_end);  // check if the object o is on the line between objects
bool isObjectInDSpace(const RigidObject& o, const RigidObject& left_border,
                      const RigidObject& right_border);  // check if the object is in the D-space
Vector_t getProjectionPhi(const Vector_t& p,
                          const Vector_t& q);  // get projection of vector p on the line orthogonal to q
bool isObjectInTSpace(const RigidObject& m, const RigidObject& i,
                      const RigidObject& j);  // check if (i,j,m) forms T set

#endif  // PROJECT_DAKAI_ALGO_H
