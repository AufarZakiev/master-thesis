#include <tf/transform_listener.h>
#include "../include/headers/helper_functions.h"

class SwarmCoordinator
{
public:
  explicit SwarmCoordinator(const ValidatedVariables& vv) : vv_(vv)
  {
    ros::Publisher pub1 = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    auto tf1 = getTransformRobotOnWorld(1);
    auto tf2 = getTransformRobotOnWorld(2);
    auto tf3 = getTransformRobotOnWorld(3);
    auto tf4 = getTransformRobotOnWorld(4);
    Robot r1(Position_t(tf1.getOrigin().getX(), tf1.getOrigin().getY()), 1);
    Robot r2(Position_t(tf2.getOrigin().getX(), tf2.getOrigin().getY()), 2);
    Robot r3(Position_t(tf3.getOrigin().getX(), tf3.getOrigin().getY()), 3);
    Robot r4(Position_t(tf4.getOrigin().getX(), tf4.getOrigin().getY()), 4);
    ROS_INFO_STREAM("r1: " << r1.getPosition());
    ROS_INFO_STREAM("r2: " << r2.getPosition());
    ROS_INFO_STREAM("r3: " << r3.getPosition());
    ROS_INFO_STREAM("r4: " << r4.getPosition());


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
    bool gotTF = false;
    while (!gotTF)
    {
      try
      {
        listener.lookupTransform("world", "swarmbot" + std::to_string(id) + "/link_chassis", ros::Time(0), transform);
        gotTF = true;
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }
    }
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

  SwarmCoordinator sc(vv);

  return 0;
}
