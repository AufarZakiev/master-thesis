#include "../include/headers/helper_functions.h"

std::pair<Obstacle, double> closestObstacleToLOS(const Robot& i, const Robot& j,
                                                 const ObstacleGraph& detected_obstacle_graph_in_D_set)
{
  Vector_t ji = getRelativePosition(i, j);
  auto min = std::numeric_limits<double>::max();
  Obstacle min_obstacle = detected_obstacle_graph_in_D_set[0];
  for (auto id = 0; id < boost::num_vertices(detected_obstacle_graph_in_D_set); ++id)
  {
    if (getVectorLength(getProjectionPhi(getRelativePosition(i, detected_obstacle_graph_in_D_set[id]), ji)) < min)
    {
      min = getVectorLength(getProjectionPhi(getRelativePosition(i, detected_obstacle_graph_in_D_set[id]), ji));
      min_obstacle = detected_obstacle_graph_in_D_set[id];
    }
  }
  return std::make_pair(min_obstacle, min);
}

std::pair<Obstacle, double> closestObstacleToLOSatFront(const Robot& i, const Robot& j,
                                                        const ObstacleGraph& closing_obstacles_in_D_space)
{
  Vector_t ji = getRelativePosition(i, j);
  auto min = std::numeric_limits<double>::max();
  Obstacle min_obstacle = closing_obstacles_in_D_space[0];
  for (auto id = 0; id < boost::num_vertices(closing_obstacles_in_D_space); ++id)
  {
    if (getVectorLength(getProjectionPhi(getRelativePosition(i, closing_obstacles_in_D_space[id]), ji)) < min)
    {
      min = getVectorLength(getProjectionPhi(getRelativePosition(i, closing_obstacles_in_D_space[id]), ji));
      min_obstacle = closing_obstacles_in_D_space[id];
    }
  }
  return std::make_pair(min_obstacle, min);
}

Robot j_star_compute(const Robot& i, const RobotGraph& robots_near_preserved,
                     const ObstacleGraph& detected_obstacle_graph_in_D_set)
{
  auto min = closestObstacleToLOS(i, robots_near_preserved[0], detected_obstacle_graph_in_D_set);
  auto min_id = 0;
  for (auto id = 0; id < boost::num_vertices(detected_obstacle_graph_in_D_set); ++id)
  {
    if (closestObstacleToLOS(i, robots_near_preserved[id], detected_obstacle_graph_in_D_set).second < min.second)
    {
      min = closestObstacleToLOS(i, robots_near_preserved[id], detected_obstacle_graph_in_D_set);
      min_id = id;
    }
  }
  return robots_near_preserved[min_id];
}

std::optional<Obstacle> closestDetectedObstacle(const RigidObject& position, const ObstacleGraph& obstacles_detected,
                                                const Variables& v)
{
  // TODO: change all trivial loops and std::algorithm
  if (boost::num_vertices(obstacles_detected) == 0)
  {
    return std::nullopt;  // TODO: Find the point to filter this case
  }
  auto min = getVectorLength(getRelativePosition(obstacles_detected[0], position));
  Obstacle min_obstacle = obstacles_detected[0];
  for (auto id = 0; id < boost::num_vertices(obstacles_detected); id++)
  {
    if (getVectorLength(getRelativePosition(obstacles_detected[id], position)) < min)
    {
      min = getVectorLength(getRelativePosition(obstacles_detected[id], position));
      min_obstacle = obstacles_detected[id];
    }
  }
  return min_obstacle;
}

std::pair<RobotGraph, RobotGraph>
separateNeighbourRobotsBehindAndFront(const Robot& robot, const RobotGraph& neighbourhood_preserved_robots)
{
  RobotGraph behind, front;
  for (auto id = 0; id < boost::num_vertices(neighbourhood_preserved_robots); id++)
  {
    if (neighbourhood_preserved_robots[id].getSpeedDirection().dot(robot.getSpeedDirection()) <= 0)
    {
      boost::add_vertex(neighbourhood_preserved_robots[id], behind);
    }
    else
    {
      boost::add_vertex(neighbourhood_preserved_robots[id], front);
    }
  }
  return std::make_pair(behind, front);
}

std::pair<RobotGraph, RobotGraph> separateDetectedRobotsBehindAndFront(const Robot& robot,
                                                                       const RobotGraph& detected_robots)
{
  RobotGraph detected_behind, detected_front;
  for (auto id = 0; id < boost::num_vertices(detected_robots); id++)
  {
    if (detected_robots[id].getSpeedDirection().dot(robot.getSpeedDirection()) <= 0)
    {
      boost::add_vertex(detected_robots[id], detected_behind);
    }
    else
    {
      boost::add_vertex(detected_robots[id], detected_front);
    }
  }
  return std::make_pair(detected_behind, detected_front);
}

ObstacleGraph closingObstaclesInDSpace(const Robot& robot_i, const Robot& robot_j, const ObstacleGraph& obstacles)
{
  ObstacleGraph result;
  for (auto id = 0; id < boost::num_vertices(obstacles); id++)
  {
    if (isObjectInDSpace(obstacles[id], robot_i, robot_j))
    {
      Vector_t oi = getRelativePosition(robot_i, obstacles[id]);
      Vector_t ji = getRelativePosition(robot_i, robot_j);
      if (robot_i.getSpeedDirection().dot(getProjectionPhi(oi, ji)) > 0)
      {
        boost::add_vertex(obstacles[id], result);
      }
    }
  }
  return result;
}

void printPlot(const std::vector<std::vector<std::tuple<double, double, double>>>& frame, const std::string& filename,
               const std::string& title, int rot_x_angle, int rot_z_angle)
{
  Gnuplot gp;
  gp << "set term png\n";
  gp << "set output \"";
  gp << filename.c_str();
  gp << "\"\n";
  gp << "set view ";
  gp << rot_x_angle;
  gp << ", ";
  gp << rot_z_angle;
  gp << ", 1, 1\n";
  gp << "set samples 120, 120\n";
  gp << "set style data lines\n";
  gp << "set pm3d\n";
  gp << "set title \"";
  gp << title.c_str();
  gp << "\"\n";
  gp << "set xlabel \"X axis\"\n";
  gp << "set xlabel  offset character -3, -2, 0 font \"\" textcolor lt -1 norotate\n";
  gp << "set ylabel \"Y axis\"\n";
  gp << "set ylabel  offset character 3, -2, 0 font \"\" textcolor lt -1 rotate\n";
  gp << "set zlabel \"Z axis\"\n";
  gp << "set zlabel  offset character -5, 0, 0 font \"\" textcolor lt -1 norotate\n";
  // gp << "set zrange [ -1.00000 : 10.00000 ] noreverse nowriteback\n";
  gp << "splot [0:15] [0:15] '-' \n";
  gp.send2d(frame);
  gp.flush();
}

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