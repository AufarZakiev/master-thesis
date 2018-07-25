#include "../include/headers/DakaiAlgo.h"

int main(int argc, char** argv)
{
  // Initiate ROS and needed vars

  ros::init(argc, argv, "follow_leader_node");
  ros::NodeHandle n_("~");

  Variables& v = Variables::getInstance();

  getNotifiedParam(n_, "robots_count", v);
  getNotifiedParam(n_, "robots_distance", v);
  getNotifiedParam(n_, "obstacle_avoidance_distance", v);
  getNotifiedParam(n_, "sensing_distance",v);
  getNotifiedParam(n_, "los_clearance_distance", v);
  getNotifiedParam(n_, "neighbourhood_distance", v);
  getNotifiedParam(n_, "edge_deletion_distance", v);

  return 0;
}