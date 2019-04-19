#include <tf/transform_listener.h>
#include "../include/headers/helper_functions.h"

class SwarmCoordinator
{
public:
  explicit SwarmCoordinator(const ValidatedVariables& vv) : vv_(vv)
  {
    ros::Publisher pub1 = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  }

private:
  ValidatedVariables vv_;
  tf::TransformListener listener;
  ros::NodeHandle n;

  tf::StampedTransform getTransformBetweenRobots(int id1, int id2)
  {
    tf::StampedTransform transform;
    listener.lookupTransform("/swarmbot" + std::to_string(id1) + "/odom", "/swarmbot" + std::to_string(id2) + "odom",
                             ros::Time(0), transform);
    return transform;
  }

  tf::StampedTransform getTransformRobotOnWorld(int id)
  {
    tf::StampedTransform transform;
    listener.lookupTransform("/swarmbot" + std::to_string(id) + "/odom", "world", ros::Time(0), transform);
    return transform;
  }
};

int main(int argc, char** argv)
{
  // Initiate ROS and needed vars

  ros::init(argc, argv, "follow_leader_node");
  ros::NodeHandle n_("~");

  Variables v = Variables();

  getNotifiedParam(n_, "robots_avoidance_distance", v);
  getNotifiedParam(n_, "obstacles_avoidance_distance", v);
  getNotifiedParam(n_, "los_clearance_distance", v);
  getNotifiedParam(n_, "los_clearance_care_distance", v);
  getNotifiedParam(n_, "neighbourhood_distance", v);
  getNotifiedParam(n_, "edge_deletion_distance", v);
  getNotifiedParam(n_, "obstacle_care_distance", v);
  getNotifiedParam(n_, "desired_distance", v);
  getNotifiedParam(n_, "sensing_distance", v);
  getNotifiedParam(n_, "robot_max_speed", v);
  getNotifiedParam(n_, "k1", v);
  getNotifiedParam(n_, "k2", v);
  getNotifiedParam(n_, "c1", v);
  getNotifiedParam(n_, "c2", v);
  getNotifiedParam(n_, "c3", v);
  getNotifiedParam(n_, "c4", v);

  ValidatedVariables vv(v);

  return 0;
}
