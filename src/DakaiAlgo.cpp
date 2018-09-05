#include <ros/init.h>
#include "../include/headers/DakaiAlgo.h"
#include "../include/headers/Variables.h"
#include "../include/headers/RigidObject.h"
#include "../include/headers/RobotGraph.h"

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

Position_t getRelativePosition(const RigidObject& o1, const RigidObject& o2)
{
  // get position of o2 in respect to o1
  return o2.getPosition() - o1.getPosition();
};
double get2DVectorLength(const Vector_t& v)
{
  return sqrt(v(0, 0) * v(0, 0) + v(1, 0) * v(1, 0));
};
double getVector3DLength(const Eigen::Vector3d& v)
{
  return sqrt(v(0, 0) * v(0, 0) + v(1, 0) * v(1, 0) + v(2, 0) * v(2, 0));
};
double getSquaredVectorLength(const Vector_t& v)
{
  return v(0, 0) * v(0, 0) + v(1, 0) * v(1, 0);
};
bool isEdgePreserved(const Robot& i, const Robot& j, const RobotGraph& rg, const Variables& v)
{
  for (const Robot& m : rg.robots)
  {
    if (m.getId() != i.getId() && m.getId() != j.getId())
    {
      if (isObjectInTSet(i, j, m, rg, v) || isObjectInTSet(j, i, m, rg, v) || isObjectInDashedTSet(i, j, m, rg, v) ||
          isObjectInDashedTSet(j, i, m, rg, v))
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
  bool isPointOnLine = (getVector3DLength(o_s_3d.cross(o_e_3d)) == 0.0);
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

double angleBetweenVectorsInRadians(const Vector_t& v1, const Vector_t& v2)
{
  double alpha = atan2(v2(1, 0), v2(0, 0)) - atan2(v1(1, 0), v1(0, 0));
  return alpha;
}

bool isObjectInTSet(const Robot& i, const Robot& j, const Robot& m, const RobotGraph& rg,
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
  bool isPhiLessThanDeletionDistance = (get2DVectorLength(getProjectionPhi(mi, ji)) <= EDGE_DELETION_DISTANCE);
  bool isMPointInDSpace = isObjectInDSpace(m, i, j);
  bool areAllVectorsInGraph = rg.isEdgeExist(i, j) && rg.isEdgeExist(j, m) && rg.isEdgeExist(m, i);
  bool isAngleBetweenVectorsGreaterThanZero = angleBetweenVectorsInRadians(im, jm) > 0.0;
  return isAngleBetweenVectorsGreaterThanZero && isPhiLessThanDeletionDistance && isMPointInDSpace &&
         areAllVectorsInGraph;
}

bool isObjectInDashedTSet(const Robot& i, const Robot& j, const Robot& m, const RobotGraph& rg, const Variables& v)
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
  bool areDistancesEqual = (get2DVectorLength(ji) - NEIGHBOURHOOD_DISTANCE) < SMALL_POSITIVE_CONSTANT &&
                           (get2DVectorLength(mj) - NEIGHBOURHOOD_DISTANCE) < SMALL_POSITIVE_CONSTANT &&
                           (get2DVectorLength(mi) - ROBOTS_AVOIDANCE_DISTANCE) < SMALL_POSITIVE_CONSTANT;
  bool areAllVectorsInGraph = rg.isEdgeExist(i, j) && rg.isEdgeExist(j, m) && rg.isEdgeExist(m, i);
  bool isAngleBetweenVectorsGreaterThanZero = angleBetweenVectorsInRadians(im, jm) > 0.0;
  return areDistancesEqual && areAllVectorsInGraph && isAngleBetweenVectorsGreaterThanZero;
}

double phiInterrobotCollisionPotential(double z, const Variables& v)
{
  double ROBOTS_AVOIDANCE_DISTANCE, NEIGHBOURHOOD_DISTANCE, DESIRED_DISTANCE, K1, K2;
  v.getParam("robots_avoidance_distance", ROBOTS_AVOIDANCE_DISTANCE);
  v.getParam("neighbourhood_distance", NEIGHBOURHOOD_DISTANCE);
  v.getParam("desired_distance", DESIRED_DISTANCE);
  v.getParam("k1", K1);
  v.getParam("k2", K2);
  double part1 = [&](double z) {
    return (z - DESIRED_DISTANCE) * (z - DESIRED_DISTANCE) * (NEIGHBOURHOOD_DISTANCE - z) /
           ((NEIGHBOURHOOD_DISTANCE - ROBOTS_AVOIDANCE_DISTANCE) *
                (NEIGHBOURHOOD_DISTANCE - ROBOTS_AVOIDANCE_DISTANCE) * (z - ROBOTS_AVOIDANCE_DISTANCE) +
            (DESIRED_DISTANCE - ROBOTS_AVOIDANCE_DISTANCE) * (DESIRED_DISTANCE - ROBOTS_AVOIDANCE_DISTANCE) *
                (NEIGHBOURHOOD_DISTANCE - z) / K1);
  }(z);
  double part2 = [&](double z) {
    return (z - DESIRED_DISTANCE) * (z - DESIRED_DISTANCE) * (z - ROBOTS_AVOIDANCE_DISTANCE) /
           ((NEIGHBOURHOOD_DISTANCE - ROBOTS_AVOIDANCE_DISTANCE) *
                (NEIGHBOURHOOD_DISTANCE - ROBOTS_AVOIDANCE_DISTANCE) * (NEIGHBOURHOOD_DISTANCE - z) +
            (ROBOTS_AVOIDANCE_DISTANCE - DESIRED_DISTANCE) * (ROBOTS_AVOIDANCE_DISTANCE - DESIRED_DISTANCE) *
                (z - ROBOTS_AVOIDANCE_DISTANCE) / K2);
  }(z);
  return part1 + part2;
}

double psiInterrobotCollisionPotential(const Robot& i, const RobotGraph& robot_graph, const Variables& v)
{
  double sum = 0;
  for (const Robot& j : robot_graph.robots)
  {
    double arg = get2DVectorLength(i.getPosition() - j.getPosition());
    sum += phiInterrobotCollisionPotential(arg, v);
  }
  return sum;
}

double phiObstacleCollisionPotential(double z, const Variables& v)
{
  double OBSTACLE_CARE_DISTANCE, OBSTACLE_AVOIDANCE_DISTANCE, SMALL_POSITIVE_CONSTANT;
  v.getParam("obstacle_care_distance", OBSTACLE_CARE_DISTANCE);
  v.getParam("obstacles_avoidance_distance", OBSTACLE_AVOIDANCE_DISTANCE);
  v.getParam("small_positive_constant", SMALL_POSITIVE_CONSTANT);
  double potential = 0;
  if (z >= OBSTACLE_CARE_DISTANCE)
    return 0;
  potential = (1.0 / ((z - OBSTACLE_AVOIDANCE_DISTANCE) / (OBSTACLE_CARE_DISTANCE - OBSTACLE_AVOIDANCE_DISTANCE)) +
               SMALL_POSITIVE_CONSTANT) -
              (1.0 / (1.0 + SMALL_POSITIVE_CONSTANT));
  potential = potential * potential / 2;
  return potential;
}

Obstacle psiObstacleCollisionHelperFunc(const Robot& i, const Obstacle_set& current_obstacle_set)
{
  Obstacle o_min = *current_obstacle_set.begin();
  for (const auto& o : current_obstacle_set)
  {
    if (get2DVectorLength(o.getPosition() - i.getPosition()) < get2DVectorLength(o_min.getPosition() - i.getPosition()))
    {
      o_min = o;
    }
  }
  return o_min;
}

double psiObstacleCollisionPotential(const Robot& i, const Obstacle_set& current_obstacle_set, const Variables& v)
{
  phiObstacleCollisionPotential(
      get2DVectorLength(i.getPosition() - psiObstacleCollisionHelperFunc(i, current_obstacle_set).getPosition()), v);
}

double phiLOSPreservePotential(double z, const Variables& v)
{
  double LOS_CLEARANCE_DISTANCE, LOS_CLEARANCE_CARE_DISTANCE, SMALL_POSITIVE_CONSTANT;
  v.getParam("los_clearance_distance", LOS_CLEARANCE_DISTANCE);
  v.getParam("los_clearance_care_distance", LOS_CLEARANCE_CARE_DISTANCE);
  v.getParam("small_positive_constant", SMALL_POSITIVE_CONSTANT);
  double potential = 0;
  if (z >= LOS_CLEARANCE_CARE_DISTANCE)
    return 0;
  potential = (1.0 / ((z - LOS_CLEARANCE_DISTANCE) / (LOS_CLEARANCE_CARE_DISTANCE - LOS_CLEARANCE_DISTANCE)) +
               SMALL_POSITIVE_CONSTANT) -
              (1.0 / (1.0 + SMALL_POSITIVE_CONSTANT));
  potential = potential * potential / 2;
  return potential;
}

Obstacle psiLOSPreserveHelperFunc(const Robot& i, const Robot& current_j, const Obstacle_set& current_obstacle_set_in_D)
{
  Obstacle o_min = *current_obstacle_set_in_D.begin();
  for (const auto& o : current_obstacle_set_in_D)
  {
    if (get2DVectorLength(getProjectionPhi(getRelativePosition(i, o), getRelativePosition(i, current_j))) <
        get2DVectorLength(getProjectionPhi(getRelativePosition(i, o_min), getRelativePosition(i, current_j))))
    {
      o_min = o;
    }
  }
  return o_min;
}

Robot psiLOSPreserveHelperFunc2(const Robot& i, const Obstacle_set& i_obstacle_set,
                                const Robot_set& neighbourhood_robots_sigma)
{
  Robot r_min = *neighbourhood_robots_sigma.begin();
  for (const auto& r : neighbourhood_robots_sigma)
  {
    if (get2DVectorLength(psiLOSPreserveHelperFunc(i, r, i_obstacle_set).getPosition()) <
        get2DVectorLength(psiLOSPreserveHelperFunc(i, r_min, i_obstacle_set).getPosition()))
    {
      r_min = r;
    }
  }
  return r_min;
}

double psiLOSPreservePotential(const Robot& i, const Obstacle_set& i_obstacle_set,
                               const Robot_set& i_neighbourhood_sigma_set,
                               const Obstacle_set& current_obstacle_set_in_D, const Variables& v)
{
  Robot j_star = psiLOSPreserveHelperFunc2(i, i_obstacle_set, i_neighbourhood_sigma_set);
  return phiLOSPreservePotential(
      get2DVectorLength(getProjectionPhi(
          i.getPosition() - psiLOSPreserveHelperFunc(i, j_star, current_obstacle_set_in_D).getPosition(),
          i.getPosition() - j_star.getPosition())),
      v);
}

double phiCohesionPotential(double z, const Variables& v)
{
  double NEIGHBOURHOOD_DISTANCE;
  v.getParam("neighbourhood_distance", NEIGHBOURHOOD_DISTANCE);
  double potential = 0;
  if (z <= NEIGHBOURHOOD_DISTANCE)
    return 0;
  potential = (z - NEIGHBOURHOOD_DISTANCE) * (z - NEIGHBOURHOOD_DISTANCE) / 2;
  return potential;
}

double psiCohesionPotential(const Robot& i, const Robot_set& sensed_robots_set, const Variables& v)
{
  double sum = 0;
  for (auto& sensed : sensed_robots_set)
  {
    sum += phiCohesionPotential(get2DVectorLength(i.getPosition() - sensed.getPosition()), v);
  }
  return sum;
}

double potentialFunction(const Robot_set& all_robots, const RobotGraph& robot_graph, const Obstacle_set& i_obstacle_set,
                         const Variables& v, const Robot_set& neighbourhood_robots_sgima,
                         const Obstacle_set& i_obstacle_set_in_D, const Robot_set& sensed_robots_set)
{
  double C1, C2, C3, C4;
  v.getParam("c1", C1);
  v.getParam("c2", C2);
  v.getParam("c3", C3);
  v.getParam("c4", C4);

  double potential = 0;
  for (auto& i : all_robots)
  {
    potential += C1 * psiInterrobotCollisionPotential(i, robot_graph, v) +
                 C2 * psiObstacleCollisionPotential(i, i_obstacle_set, v) +
                 C3 * psiLOSPreservePotential(i, i_obstacle_set, neighbourhood_robots_sgima, i_obstacle_set_in_D, v) +
                 C4 * psiCohesionPotential(i, sensed_robots_set, v);
  }

  return potential;
}