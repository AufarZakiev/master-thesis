#include "../include/headers/dakai_algo.h"

int main(int argc, char** argv)
{
  // Initiate ROS and needed vars

  ros::init(argc, argv, "maps_union_node");
  ros::NodeHandle n_;

  getNotifiedParam(n_, "robots_count", constants::ROBOTS_COUNT);
  getNotifiedParam(n_, "robots_distance", constants::ROBOTS_AVOIDANCE_DISTANCE);
  getNotifiedParam(n_, "obstacle_avoidance_distance", constants::OBSTACLES_AVOIDANCE_DISTANCE);
  getNotifiedParam(n_, "sensing_distance", constants::SENSING_DISTANCE);
  getNotifiedParam(n_, "los_clearance_distance", constants::LOS_CLEARANCE_DISTANCE);
  getNotifiedParam(n_, "neighbourhood_distance", constants::NEIGHBOURHOOD_DISTANCE);
  getNotifiedParam(n_, "edge_deletion_distance", constants::EDGE_DELETION_DISTANCE);

  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}