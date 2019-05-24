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

    //    boost::add_vertex(Obstacle(Position_t(-0.5, -6.5), 0.5), *og);
    //    boost::add_vertex(Obstacle(Position_t(-1.5, -7.5), 0.5), *og);
    //    boost::add_vertex(Obstacle(Position_t(-2.5, -8.5), 0.5), *og);
    //    boost::add_vertex(Obstacle(Position_t(-3.5, -9.5), 0.5), *og);
    //    boost::add_vertex(Obstacle(Position_t(-4.5, -10.5), 0.5), *og);
    //    boost::add_vertex(Obstacle(Position_t(-5.5, -11.5), 0.5), *og);
    //
    //    boost::add_vertex(Obstacle(Position_t(-6.5, -0.5), 0.5), *og);
    //    boost::add_vertex(Obstacle(Position_t(-7.5, -1.5), 0.5), *og);
    //    boost::add_vertex(Obstacle(Position_t(-8.5, -2.5), 0.5), *og);
    //    boost::add_vertex(Obstacle(Position_t(-9.5, -3.5), 0.5), *og);
    //    boost::add_vertex(Obstacle(Position_t(-10.5, -4.5), 0.5), *og);
    //    boost::add_vertex(Obstacle(Position_t(-11.5, -5.5), 0.5), *og);

    boost::add_vertex(Obstacle(Position_t(-0.5, -6.5), 2.0), *og);
    boost::add_vertex(Obstacle(Position_t(-1.5, -7.5), 2.0), *og);
    boost::add_vertex(Obstacle(Position_t(-2.5, -8.5), 2.0), *og);
    boost::add_vertex(Obstacle(Position_t(-3.5, -9.5), 2.0), *og);
    boost::add_vertex(Obstacle(Position_t(-4.5, -10.5), 2.0), *og);
    boost::add_vertex(Obstacle(Position_t(-5.5, -11.5), 2.0), *og);

    boost::add_vertex(Obstacle(Position_t(-6.5, -0.5), 2.0), *og);
    boost::add_vertex(Obstacle(Position_t(-7.5, -1.5), 2.0), *og);
    boost::add_vertex(Obstacle(Position_t(-8.5, -2.5), 2.0), *og);
    boost::add_vertex(Obstacle(Position_t(-9.5, -3.5), 2.0), *og);
    boost::add_vertex(Obstacle(Position_t(-10.5, -4.5), 2.0), *og);
    boost::add_vertex(Obstacle(Position_t(-11.5, -5.5), 2.0), *og);

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
      // ROS_INFO_STREAM("Diff: " << getVectorLength(diff));
      if (diff.dot(needDirection) >= 0.0)
      {
        geometry_msgs::Twist command;
        command.linear.x = getVectorLength(diff) / 2.0;
        swarm_pubs[id].publish(command);
      }
      else
      {
        break;
      }
    } while (getVectorLength(diff) > 0.02);

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

  //  getNotifiedParam(n_, "robots_avoidance_distance", v);
  //  getNotifiedParam(n_, "obstacles_avoidance_distance", v);
  //  getNotifiedParam(n_, "los_clearance_distance", v);
  //  getNotifiedParam(n_, "los_clearance_care_distance", v);
  //  getNotifiedParam(n_, "neighbourhood_distance", v);
  //  getNotifiedParam(n_, "edge_deletion_distance", v);
  //  getNotifiedParam(n_, "obstacle_care_distance", v);
  //  getNotifiedParam(n_, "desired_distance", v);
  //  getNotifiedParam(n_, "sensing_distance", v);
  //  getNotifiedParam(n_, "robot_max_speed", v);
  //  getNotifiedParam(n_, "k1", v);
  //  getNotifiedParam(n_, "k2", v);
  //  getNotifiedParam(n_, "c1", v);
  //  getNotifiedParam(n_, "c2", v);
  //  getNotifiedParam(n_, "c3", v);
  //  getNotifiedParam(n_, "c4", v);

  v.setParam("robots_avoidance_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.7);
  v.setParam("los_clearance_distance", 0.1);
  v.setParam("los_clearance_care_distance", 6.5);
  v.setParam("neighbourhood_distance", 10.0);
  v.setParam("edge_deletion_distance", 2.5);
  v.setParam("obstacle_care_distance", 6.5);
  v.setParam("desired_distance", 6.5);
  v.setParam("sensing_distance", 10.5);
  v.setParam("robot_max_speed", 0.5);
  v.setParam("k1", 10);
  v.setParam("k2", 10);
  v.setParam("c1", 2.5);
  v.setParam("c2", 1.25);
  v.setParam("c3", 0.1);
  v.setParam("c4", 10.0);

  ValidatedVariables vv(v);

  SwarmCoordinator sc(vv);

  ros::spin();
  return 0;
}
