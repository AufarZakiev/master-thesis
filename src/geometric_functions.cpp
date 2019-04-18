#include "../include/headers/geometric_functions.h"

std::optional<RobotDesc> findRobotInGraph(const Robot& ro, const RobotGraph& graph)
{
  for (RobotDesc desc = 0; desc < boost::num_vertices(graph); ++desc)
  {
    if (graph[desc].getRobotID() == ro.getRobotID())
      return desc;
  }
  return std::nullopt;
};

std::optional<ObstacleDesc> findObstacleInGraph(const Obstacle& ro, const ObstacleGraph& graph)
{
  for (ObstacleDesc desc = 0; desc < boost::num_vertices(graph); ++desc)
  {
    if (graph[desc].getObstacleID() == ro.getObstacleID())
      return desc;
  }
  return std::nullopt;
};

double getVectorDistance(const Vector_t& v1, const Vector_t& v2)
{
  double x_coord = v1(0, 0) - v2(0, 0);
  double y_coord = v1(1, 0) - v2(1, 0);
  return sqrt(x_coord * x_coord + y_coord * y_coord);
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
bool isEdgePreserved(const Robot& i, const Robot& j, const RobotGraph& robots, const Variables& v)
{
  for (RobotDesc id = 0; id < boost::num_vertices(robots); ++id)
  {
    if (robots[id].getRobotID() != i.getRobotID() && robots[id].getRobotID() != j.getRobotID())
    {
      if (isObjectInTSet(i, j, robots[id], v) || isObjectInTSet(j, i, robots[id], v) ||
          isObjectInDashedTSet(i, j, robots[id], v) || isObjectInDashedTSet(j, i, robots[id], v))
        return false;
    }
  }
  return true;
};
bool isObjectOnLineSegment(const RigidObject& o, const RigidObject& line_start, const RigidObject& line_end,
                           const Variables& v)
{
  double EQUALITY_CASE;
  v.getParam("equality_case", EQUALITY_CASE);
  // check if the object o is on the line between objects
  Position_t o_s = getRelativePosition(o, line_start);
  Position_t o_e = getRelativePosition(o, line_end);
  Eigen::Vector3d o_s_3d(o_s(0, 0), o_s(1, 0), 0);
  Eigen::Vector3d o_e_3d(o_e(0, 0), o_e(1, 0), 0);
  bool isPointOnLine = (getVectorLength(o_s_3d.cross(o_e_3d)) < EQUALITY_CASE);
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

bool isEdgeInGraph(const Robot& i, const Robot& j, const RobotGraph& rg)
{
  auto i_desc = findRobotInGraph(i, rg);
  auto j_desc = findRobotInGraph(j, rg);
  if (!(i_desc && j_desc))
    return false;                                                 // check if the vertices exist
  return boost::edge(i_desc.value(), j_desc.value(), rg).second;  // check if the edge between vertices exist
}

double angleBetweenVectorsInRadians(const Vector_t& v1, const Vector_t& v2)
{
  double alpha = atan2(v2(1, 0), v2(0, 0)) - atan2(v1(1, 0), v1(0, 0));
  if (alpha > M_PI)
  {
    alpha -= 2 * M_PI;
  }
  else if (alpha <= -M_PI)
  {
    alpha += 2 * M_PI;
  }
  return alpha;
}

bool isObjectInTSet(const Robot& i, const Robot& j, const Robot& m,
                    const Variables& v)  // three objects to check and graph with edges chosen to be saved
{
  // check if (i,j,m) forms T set
  Vector_t mi = getRelativePosition(i, m);
  Vector_t im = getRelativePosition(m, i);
  Vector_t jm = getRelativePosition(m, j);
  Vector_t ji = getRelativePosition(i, j);
  double EDGE_DELETION_DISTANCE;
  v.getParam("edge_deletion_distance", EDGE_DELETION_DISTANCE);
  bool isPhiLessThanDeletionDistance = (getVectorLength(getProjectionPhi(mi, ji)) <= EDGE_DELETION_DISTANCE);
  bool isMPointInDSpace = isObjectInDSpace(m, i, j);
  //  bool areAllRobotsInGraph = isEdgeInGraph(i, j, rg) && isEdgeInGraph(j, m, rg) && isEdgeInGraph(m, i, rg);
  bool isAngleBetweenVectorsGreaterThanZero = angleBetweenVectorsInRadians(im, jm) > 0.0;
  return isAngleBetweenVectorsGreaterThanZero && isPhiLessThanDeletionDistance &&
         isMPointInDSpace;  // && areAllRobotsInGraph;
}

bool isObjectInDashedTSet(const Robot& i, const Robot& j, const Robot& m, const Variables& v)
{
  Vector_t mi = getRelativePosition(i, m);
  Vector_t im = getRelativePosition(m, i);
  Vector_t mj = getRelativePosition(j, m);
  Vector_t jm = getRelativePosition(m, j);
  Vector_t ji = getRelativePosition(i, j);
  double NEIGHBOURHOOD_DISTANCE, ROBOTS_AVOIDANCE_DISTANCE;
  v.getParam("neighbourhood_distance", NEIGHBOURHOOD_DISTANCE);
  v.getParam("robots_avoidance_distance", ROBOTS_AVOIDANCE_DISTANCE);
  double EQUALITY_CASE_1 = NEIGHBOURHOOD_DISTANCE * 0.1;
  double EQUALITY_CASE_2 = ROBOTS_AVOIDANCE_DISTANCE * 0.2;
  bool areDistancesEqual = getVectorLength(ji) <= NEIGHBOURHOOD_DISTANCE &&
                           NEIGHBOURHOOD_DISTANCE - EQUALITY_CASE_1 <= getVectorLength(ji) &&
                           getVectorLength(mj) <= NEIGHBOURHOOD_DISTANCE &&
                           NEIGHBOURHOOD_DISTANCE - EQUALITY_CASE_1 <= getVectorLength(mj) &&
                           getVectorLength(mi) <= ROBOTS_AVOIDANCE_DISTANCE + EQUALITY_CASE_2 &&
                           ROBOTS_AVOIDANCE_DISTANCE <= getVectorLength(mi);
  //  bool areAllRobotsInGraph = isEdgeInGraph(i, j, rg) && isEdgeInGraph(j, m, rg) && isEdgeInGraph(m, i, rg);
  bool isAngleBetweenVectorsGreaterThanZero = angleBetweenVectorsInRadians(im, jm) > 0.0;
  if (areDistancesEqual && isAngleBetweenVectorsGreaterThanZero)  // && areAllRobotsInGraph
  {
    std::cout << "ObjectInDashedTSet: " << i.getRobotID() << ", " << j.getRobotID() << ", " << m.getRobotID()
              << std::endl;
  }
  return areDistancesEqual && isAngleBetweenVectorsGreaterThanZero;  // && areAllRobotsInGraph
}