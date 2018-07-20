#include "../include/headers/dakai_algo.h"

int main(int argc, char** argv)
{
  // Initiate ROS and needed vars
  ros::init(argc, argv, "maps_union_node");
  ros::NodeHandle n_;
  int ROBOTS_COUNT = 2;  // TODO: default values from paper
  double ROBOTS_AVOIDANCE_DISTANCE;
  double OBSTACLES_AVOIDANCE_DISTANCE;
  double SENSING_DISTANCE;
  double LOS_CLEARANCE_DISTANCE;
  double NEIGHBOURHOOD_DISTANCE;
  double EDGE_DELETION_DISTANCE;

  getNotifiedParam(n_, "robots_count", ROBOTS_COUNT);
  getNotifiedParam(n_, "robots_distance", ROBOTS_AVOIDANCE_DISTANCE);
  getNotifiedParam(n_, "obstacle_avoidance_distance", OBSTACLES_AVOIDANCE_DISTANCE);
  getNotifiedParam(n_, "sensing_distance", SENSING_DISTANCE);
  getNotifiedParam(n_, "los_clearance_distance", LOS_CLEARANCE_DISTANCE);
  getNotifiedParam(n_, "neightbourhood_distance", NEIGHBOURHOOD_DISTANCE);
  getNotifiedParam(n_, "edge_deletion_distance", EDGE_DELETION_DISTANCE);

  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}