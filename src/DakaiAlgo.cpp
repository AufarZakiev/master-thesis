#include <ros/init.h>
#include "../include/headers/DakaiAlgo.h"
#include "../include/headers/Variables.h"
#include <tuple>
#include <utility>

void getNotifiedParam(ros::NodeHandle& n_, const std::string& param_name, Variables& v)
{
  double param_variable;
  if (n_.getParam(param_name, param_variable))
  {
    ROS_INFO("Got param %s: %f", param_name.c_str(), param_variable);
    v.setParam(param_name, param_variable);
  }
  else
  {
    ROS_ERROR("Failed to get param %s. Setting to default value", param_name.c_str());
  }
}

double getVectorDistance(const Vector_t& v1, const Vector_t& v2)
{
  double x_coord = v1(0, 0) - v2(0, 0);
  double y_coord = v1(1, 0) - v2(1, 0);
  return sqrt(x_coord * x_coord + y_coord * y_coord);
}

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

std::pair<RigidObjectDesc, bool> findVertexInGraph(const RigidObject& ro, const RigidGraph& graph)
{
  for (RigidObjectDesc id = 0; id < boost::num_vertices(graph); ++id)
  {
    if (graph[id].getPosition() == ro.getPosition())
      return std::make_pair(id, true);
  }
  return std::make_pair(0, false);
};

Robot::Robot(Position_t position) : RigidObject(position)
{
}

double Robot::getUmax() const
{
  return u_max_;
}

Position_t getRelativePosition(const RigidObject& o1, const RigidObject& o2)
{
  // get position of o2 in respect to o1
  return o2.getPosition() - o1.getPosition();
};
double getVectorLength(const Vector_t& v)
{
  return sqrt(v(0, 0) * v(0, 0) + v(1, 0) * v(1, 0));
};
double getVectorLength(const Eigen::Vector3d& v)
{
  return sqrt(v(0, 0) * v(0, 0) + v(1, 0) * v(1, 0) + v(2, 0) * v(2, 0));
};
double getSquaredVectorLength(const Vector_t& v)
{
  return v(0, 0) * v(0, 0) + v(1, 0) * v(1, 0);
};
bool isEdgePreserved(const Robot& i, const Robot& j, const RigidGraph& rg, const Variables& v)
{
  for (RigidObjectDesc id = 0; id < boost::num_vertices(rg); ++id)  // TODO: check if this can be done through hashes
  {
    if (rg[id].getPosition() != i.getPosition() && rg[id].getPosition() != j.getPosition())
    {
      if (isObjectInTSet(i, j, rg[id], rg, v) || isObjectInTSet(j, i, rg[id], rg, v) ||
          isObjectInDashedTSet(i, j, rg[id], rg, v) || isObjectInDashedTSet(j, i, rg[id], rg, v))
        return false;
    }
  }
  return true;
};
bool isObjectOnLineSegment(const RigidObject& o, const RigidObject& line_start, const RigidObject& line_end)
{
  // check if the object o is on the line between objects
  Position_t o_s = getRelativePosition(o, line_start);
  Position_t o_e = getRelativePosition(o, line_end);
  Eigen::Vector3d o_s_3d(o_s(0, 0), o_s(1, 0), 0);
  Eigen::Vector3d o_e_3d(o_e(0, 0), o_e(1, 0), 0);
  bool isPointOnLine = (getVectorLength(o_s_3d.cross(o_e_3d)) == 0.0);
  bool isPointBetweenSegmentEnds = (o_s.dot(o_e) <= 0.0);
  return isPointOnLine && isPointBetweenSegmentEnds;
};
bool isObjectInDSpace(const RigidObject& o, const RigidObject& left_border, const RigidObject& right_border)
{
  // check if the object is in the D-space
  Position_t o_l = getRelativePosition(left_border, o);
  Position_t o_r = getRelativePosition(right_border, o);
  Position_t r_l = getRelativePosition(left_border, right_border);
  return (o_l.dot(r_l) > 0.0) && (o_r.dot(r_l) < 0.0);
};
Vector_t getProjectionPhi(const Vector_t& p, const Vector_t& q)
{
  // get projection of vector p on the line orthogonal to q
  Eigen::Matrix2d h;
  h << 0, -1, 1, 0;
  Eigen::RowVector2d p_row;
  p_row << p(0, 0), p(1, 0);
  Vector_t temp = h * q;                                                // vector, orthogonal to q
  double scale = (p_row * h * q / getSquaredVectorLength(temp)).sum();  // scaling factor
  Vector_t f = scale * h * q;                                           // result vector
  return f;
};

bool isVectorInGraph(const RigidObject& i, const RigidObject& j, const RigidGraph& rg)
{
  RigidObjectDesc i_d, j_d;
  bool i_found, j_found;
  std::tie(i_d, i_found) = findVertexInGraph(i, rg);
  std::tie(j_d, j_found) = findVertexInGraph(j, rg);
  if (!(i_found && j_found))
    return false;                           // check if the vertices exist
  return boost::edge(i_d, j_d, rg).second;  // check if the edge between vertices exist
}

double angleBetweenVectorsInRadians(const Vector_t& v1, const Vector_t& v2)
{
  double alpha = atan2(v2(1, 0), v2(0, 0)) - atan2(v1(1, 0), v1(0, 0));
  return alpha;
}

bool isObjectInTSet(const RigidObject& i, const RigidObject& j, const RigidObject& m, const RigidGraph& rg,
                    const Variables& v)  // three objects to check and graph with edges chosen to be saved
{
  // check if (i,j,m) forms T set
  Vector_t mi = getRelativePosition(i, m);
  Vector_t im = getRelativePosition(m, i);
  Vector_t mj = getRelativePosition(j, m);
  Vector_t jm = getRelativePosition(m, j);
  Vector_t ji = getRelativePosition(i, j);
  double EDGE_DELETION_DISTANCE;
  v.getParam("edge_deletion_distance", EDGE_DELETION_DISTANCE);
  bool isPhiLessThanDeletionDistance = (getVectorLength(getProjectionPhi(mi, ji)) <= EDGE_DELETION_DISTANCE);
  bool isMPointInDSpace = isObjectInDSpace(m, i, j);
  bool areAllVectorsInGraph = isVectorInGraph(i, j, rg) && isVectorInGraph(j, m, rg) && isVectorInGraph(m, i, rg);
  bool isAngleBetweenVectorsGreaterThanZero = angleBetweenVectorsInRadians(im, jm) > 0.0;
  return isAngleBetweenVectorsGreaterThanZero && isPhiLessThanDeletionDistance && isMPointInDSpace &&
         areAllVectorsInGraph;
}

bool isObjectInDashedTSet(const RigidObject& i, const RigidObject& j, const RigidObject& m, const RigidGraph& rg,
                          const Variables& v)
{
  Vector_t mi = getRelativePosition(i, m);
  Vector_t im = getRelativePosition(m, i);
  Vector_t mj = getRelativePosition(j, m);
  Vector_t jm = getRelativePosition(m, j);
  Vector_t ji = getRelativePosition(i, j);
  double NEIGHBOURHOOD_DISTANCE, ROBOTS_AVOIDANCE_DISTANCE, SMALL_POSITIVE_CONSTANT;
  v.getParam("neighbourhood_distance", NEIGHBOURHOOD_DISTANCE);
  v.getParam("robots_avoidance_distance", ROBOTS_AVOIDANCE_DISTANCE);
  v.getParam("small_positive_constant", SMALL_POSITIVE_CONSTANT);
  bool areDistancesEqual = (getVectorLength(ji) - NEIGHBOURHOOD_DISTANCE) < SMALL_POSITIVE_CONSTANT &&
                           (getVectorLength(mj) - NEIGHBOURHOOD_DISTANCE) < SMALL_POSITIVE_CONSTANT &&
                           (getVectorLength(mi) - ROBOTS_AVOIDANCE_DISTANCE) < SMALL_POSITIVE_CONSTANT;
  bool areAllVectorsInGraph = isVectorInGraph(i, j, rg) && isVectorInGraph(j, m, rg) && isVectorInGraph(m, i, rg);
  bool isAngleBetweenVectorsGreaterThanZero = angleBetweenVectorsInRadians(im, jm) > 0.0;
  return areDistancesEqual && areAllVectorsInGraph && isAngleBetweenVectorsGreaterThanZero;
}