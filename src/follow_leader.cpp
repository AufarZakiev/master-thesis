#include <tf/transform_listener.h>
#include "../include/headers/helper_functions.h"

class SwarmCoordinator
{
public:
  explicit SwarmCoordinator(const ValidatedVariables& vv) : vv_(vv)
  {
    //    ros::Subscriber sub =
    //        n.subscribe("/tf", 1, &SwarmCoordinator::tfCallback, this, ros::TransportHints().tcpNoDelay());

    const int N = 4;
    swarm_pubs.resize(N + 1);
    for (int i = 1; i <= N; i++)
    {
      swarm_pubs[i] = n.advertise<geometry_msgs::Twist>("swarmbot" + std::to_string(i) + "/cmd_vel", 1000);
    }

    while (ros::ok())
    {
      tick();
      ros::Duration(0.001).sleep();
    }
  }

private:
  ValidatedVariables vv_;
  tf::TransformListener listener;
  ros::NodeHandle n;
  std::vector<ros::Publisher> swarm_pubs;

  void tick()
  {
    const int N = 4;
    auto rg = std::make_unique<RobotGraph>();
    for (int i = 1; i <= N; i++)
    {
      auto tf = getTransformRobotOnWorld(i);

      Robot temp(Position_t(tf.getOrigin().getX(), tf.getOrigin().getY()), i);
      double yaw, pitch, roll;
      tf::Matrix3x3 mat(tf.getRotation());
      mat.getEulerYPR(yaw, pitch, roll);

      temp.setSpeedDirection(Vector_t(cos(yaw), sin(yaw)));

      boost::add_vertex(temp, *rg);
    }

    auto og = std::make_unique<ObstacleGraph>();
    Vector_t needDirection(-1, -1);

    ValidatedGraphs vg(std::move(rg), std::move(og), vv_);

    vg.tickGazebo(1, needDirection, vv_);

    for (int i = 1; i <= N; i++)
    {
      rotateTo(vg.getRobotGraph()[i - 1].getSpeedDirection(), i);
      moveTo(vg.getRobotGraph()[i - 1].getSpeedDirection(), i);
    }
  }

  void rotateTo(const Vector_t& needDirection, int id)
  {
    ROS_INFO_STREAM("Needed direction of robot " << id << " is " << needDirection(0, 0) << ";" << needDirection(1, 0));
    double angle;
    do
    {
      double yaw, pitch, roll;
      tf::Matrix3x3 mat(getTransformRobotOnWorld(id).getRotation());
      mat.getEulerYPR(yaw, pitch, roll);

      //      ROS_INFO_STREAM("Yaw: " << yaw);
      //      ROS_INFO_STREAM("Pitch: " << pitch);  // no sense here
      //      ROS_INFO_STREAM("Roll: " << roll);    // no sense here

      angle = angleBetweenVectorsInRadians(needDirection, Vector_t(cos(yaw), sin(yaw)));
      // ROS_INFO_STREAM("Angle: " << angle);
      if (angle > 0.0)
      {
        geometry_msgs::Twist command;
        command.angular.z = -0.5;
        swarm_pubs[id].publish(command);
      }
      else
      {
        geometry_msgs::Twist command;
        command.angular.z = 0.5;
        swarm_pubs[id].publish(command);
      }
    } while (abs(angle) > 0.01);

    {
      geometry_msgs::Twist command;
      command.angular.z = 0.0;
      swarm_pubs[id].publish(command);
    }
  }

  void moveTo(const Vector_t& needDirection, int id)
  {
    Vector_t start =
        Vector_t(getTransformRobotOnWorld(id).getOrigin().getX(), getTransformRobotOnWorld(id).getOrigin().getY());
    auto end = start + needDirection;
    Vector_t diff;
    do
    {
      diff = (end - Vector_t(getTransformRobotOnWorld(id).getOrigin().getX(),
                             getTransformRobotOnWorld(id).getOrigin().getY()));
      // ROS_INFO_STREAM("Diff: " << diff);
      if (diff.dot(needDirection) > 0.0)
      {
        geometry_msgs::Twist command;
        command.linear.x = (abs(diff(0, 0)) > 0.01) ? 0.1 : 0.05;
        swarm_pubs[id].publish(command);
      }
      else
      {
        geometry_msgs::Twist command;
        command.linear.x = -0.01;
        swarm_pubs[id].publish(command);
      }
    } while (abs(diff(0, 0)) > 0.01);

    {
      geometry_msgs::Twist command;
      command.linear.x = 0.0;
      swarm_pubs[id].publish(command);
    }
  }

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
