#include <tf/transform_listener.h>
#include "../include/headers/helper_functions.h"

class SwarmCoordinator
{
public:
  explicit SwarmCoordinator(const ValidatedVariables& vv) : vv_(vv)
  {
    //    ros::Subscriber sub =
    //        n.subscribe("/tf", 1, &SwarmCoordinator::tfCallback, this, ros::TransportHints().tcpNoDelay());

    pub1 = n.advertise<geometry_msgs::Twist>("swarmbot1/cmd_vel", 1000);

    while (ros::ok())
    {
      tfCallback();
      ros::Duration(0.1).sleep();
    }

    // auto angle = angleBetweenVectorsInRadians()
    //    Robot r2(Position_t(tf_array[2].getOrigin().getX(), tf_array[2].getOrigin().getY()), 2);
    //    Robot r3(Position_t(tf_array[3].getOrigin().getX(), tf_array[3].getOrigin().getY()), 3);
    //    Robot r4(Position_t(tf_array[4].getOrigin().getX(), tf_array[4].getOrigin().getY()), 4);
    //    ROS_INFO_STREAM("r1: " << r1.getPosition());
    //    ROS_INFO_STREAM("r2: " << r2.getPosition());
    //    ROS_INFO_STREAM("r3: " << r3.getPosition());
    //    ROS_INFO_STREAM("r4: " << r4.getPosition());
    //
    //    auto rg = std::make_unique<RobotGraph>();
    //    auto r1_desc = boost::add_vertex(r1, *rg);
    //    boost::add_vertex(r2, *rg);
    //    boost::add_vertex(r3, *rg);
    //    boost::add_vertex(r4, *rg);
    //
    //    auto og = std::make_unique<ObstacleGraph>();
    //
    //    ValidatedGraphs vg(std::move(rg), std::move(og), vv);
    //
    //    auto leaderV = Vector_t(sqrt(2), sqrt(2));

    //    vg.tickGazebo(r1_desc, leaderV, vv);
    //
    //    for (size_t i = 0; i < boost::num_vertices(vg.getRobotGraph()); i++)
    //    {
    //      angleBetweenVectorsInRadians()
    //      if (tf_array[i].getRotation().getAngle()<vg.getRobotGraph()[i].getSpeedDirection().)
    //        ;
    //    }
  }

  void tfCallback()
  {
    ROS_INFO_STREAM("TfCallback");
    const int N = 1;
    std::array<tf::StampedTransform, N + 1> tf_array;
    for (int i = 1; i <= N; i++)
    {
      tf_array[i] = getTransformRobotOnWorld(i);
    }
    Robot r1(Position_t(tf_array[1].getOrigin().getX(), tf_array[1].getOrigin().getY()), 1);

    double yaw, pitch, roll;
    tf::Matrix3x3 mat(tf_array[1].getRotation());
    mat.getEulerYPR(yaw, pitch, roll);

    ROS_INFO_STREAM("Yaw: " << yaw);
    ROS_INFO_STREAM("Pitch: " << pitch);  // no sense here
    ROS_INFO_STREAM("Roll: " << roll);    // no sense here

    if (yaw > 1.57)
    {
      geometry_msgs::Twist command;
      command.angular.z = -0.1;
      pub1.publish(command);
    }
    else
    {
      geometry_msgs::Twist command;
      command.angular.z = 0.1;
      pub1.publish(command);
    }
  }

private:
  ValidatedVariables vv_;
  tf::TransformListener listener;
  ros::NodeHandle n;
  ros::Publisher pub1;

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

  ros::spin();
  return 0;
}
