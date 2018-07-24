#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include "string"
#include "chrono"

inline size_t index(size_t i, size_t column, uint32_t width)
{
  return (width * i) + column;
}

void initializeUnitedMap(ros::NodeHandle& n_, int robots_number, nav_msgs::OccupancyGrid& united_map,
                         std::vector<geometry_msgs::TransformStamped>& saved_transforms,
                         geometry_msgs::Vector3& base_vector)
{
  nav_msgs::GetMap srv;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);   // listener to perform proper transforms
  geometry_msgs::TransformStamped transformStamped;  // object to store listened transforms
  ros::ServiceClient client =
      n_.serviceClient<nav_msgs::GetMap>("/turtlebot1/dynamic_map");  // Service to retrieve map from hector_mapping
  client.call(srv);
  nav_msgs::OccupancyGrid retrieved_map = srv.response.map;
  united_map.header.frame_id = "world";
  united_map.info.resolution = retrieved_map.info.resolution;
  united_map.info.origin.orientation = retrieved_map.info.origin.orientation;
  int32_t min_X_coord =
      0;  // Minimum X coordinate of cell (0.0) across all the maps relatively to 'world' frame [in cells]
  int32_t max_X_coord = 0;
  int32_t min_Y_coord = 0;
  int32_t max_Y_coord = 0;
  for (int i = 1; i < robots_number + 1; i++)
  {
    client = n_.serviceClient<nav_msgs::GetMap>("/turtlebot" + std::__cxx11::to_string(i) +
                                                "/dynamic_map");  // Service to retrieve map from hector_mapping
    client.call(srv);
    retrieved_map = srv.response.map;
    bool transformed = false;
    while (!transformed)
    {
      try
      {
        transformStamped = tfBuffer.lookupTransform("world", "robot" + std::__cxx11::to_string(i) + "_tf/map",
                                                    ros::Time(0), ros::Duration(1.0));
        transformed = true;
        ROS_INFO("Transform x: %f, Transform y: %f", transformStamped.transform.translation.x,
                 transformStamped.transform.translation.y);
        saved_transforms.emplace_back(transformStamped);
        int32_t temp = (int32_t)(round(transformStamped.transform.translation.x / united_map.info.resolution)) -
                       retrieved_map.info.width / 2;
        if (min_X_coord > temp)
        {
          min_X_coord = temp;
          base_vector.x = fabs(transformStamped.transform.translation.x / united_map.info.resolution);
        }

        temp = (int32_t)(round(transformStamped.transform.translation.x / united_map.info.resolution)) +
               retrieved_map.info.width / 2;
        if (max_X_coord < temp)
          max_X_coord = temp;

        temp = (int32_t)(round(transformStamped.transform.translation.y / united_map.info.resolution)) -
               retrieved_map.info.height / 2;
        if (min_Y_coord > temp)
        {
          min_Y_coord = temp;
          base_vector.y = fabs(transformStamped.transform.translation.y / united_map.info.resolution);
        }

        temp = (int32_t)(round(transformStamped.transform.translation.y / united_map.info.resolution)) +
               retrieved_map.info.height / 2;
        if (max_Y_coord < temp)
          max_Y_coord = temp;
      }
      catch (tf2::TransformException& ex)
      {
        ROS_ERROR("%s", ex.what());
        ros::Duration(0.1).sleep();
        continue;
      }
    }
  }
  ROS_INFO("MIN: x = %d, y = %d, MAX: x = %d, y = %d", min_X_coord, min_Y_coord, max_X_coord, max_Y_coord);
  united_map.info.width = (uint32_t)(abs(min_X_coord) + max_X_coord);
  united_map.info.height = (uint32_t)(abs(min_Y_coord) + max_Y_coord);
  united_map.info.origin.position.x = min_X_coord * united_map.info.resolution;
  united_map.info.origin.position.y = min_Y_coord * united_map.info.resolution;

  base_vector.y += +retrieved_map.info.height / 2.0;
  base_vector.x += +retrieved_map.info.width / 2.0;
};

// Node is based on the assumption that all the merged maps have same resolution and orientation

int main(int argc, char** argv)
{
  // Initiate ROS and needed vars

  ros::init(argc, argv, "maps_union_node");
  ros::NodeHandle n_("~");
  ros::Publisher pub_ = n_.advertise<nav_msgs::OccupancyGrid>("/united_map", 1);
  ros::ServiceClient client;              // client to connect to various services
  nav_msgs::GetMap srv;                   // srv object to be called
  nav_msgs::OccupancyGrid retrieved_map;  // object to store retrieved maps
  nav_msgs::OccupancyGrid united_map;     // object to be published
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);  // listener to perform proper transforms
  geometry_msgs::TransformStamped transform;          // object to store listened transforms
  int robots_number = 0;                              // count of Turtlebots
  geometry_msgs::Vector3 base_vector;  // vector between world's (0,0) point and most left-down map corner
  std::vector<geometry_msgs::TransformStamped> savedTransforms(
      1);  // robots transforms relatively to world frame storage. First cell is empty
  // get robots number
  if (n_.getParam("robots_number", robots_number))
  {
    ROS_INFO("Got param robots_number: %d", robots_number);
  }
  else
  {
    ROS_ERROR("Failed to get param 'robots_number'. Setting to default value = 2.");
    robots_number = 2;
  }

  std::chrono::time_point<std::chrono::system_clock> start, end;

  // united map initialization
  initializeUnitedMap(n_, robots_number, united_map, savedTransforms, base_vector);

  ROS_INFO("United map params{ height: %d, width: %d, origin_x: %f, origin_y: %f }", united_map.info.width,
           united_map.info.height, united_map.info.origin.position.x, united_map.info.origin.position.y);
  united_map.data = std::vector<signed char>(united_map.info.width * united_map.info.height, -1);
  // main node cycle
  int count = 0;
  while (ros::ok())
  {
    united_map.header.seq++;
    united_map.header.stamp = ros::Time::now();
    for (int r = 1; r < robots_number + 1; r++)
    {
      start = std::chrono::system_clock::now();
      client = n_.serviceClient<nav_msgs::GetMap>("/turtlebot" + std::__cxx11::to_string(r) +
                                                  "/dynamic_map");  // Service to retrieve map from hector_mapping
      client.call(srv);
      retrieved_map = srv.response.map;
      auto r_transform_x = (int)round(savedTransforms[r].transform.translation.x / united_map.info.resolution);
      auto r_transform_y = (int)round(savedTransforms[r].transform.translation.y / united_map.info.resolution);
      for (size_t i = 0; i < retrieved_map.info.height; ++i)
      {
        for (size_t j = 0; j < retrieved_map.info.width; ++j)
        {
          auto t1 = (int)round(base_vector.y);
          auto t2 = (int)round(base_vector.x);
          signed char& current_cell =
              united_map.data[index(t1 + r_transform_y - retrieved_map.info.height / 2 + i,
                                    t2 + r_transform_x - retrieved_map.info.width / 2 + j, united_map.info.width)];
          if (current_cell == -1)
            current_cell = retrieved_map.data[index(i, j, retrieved_map.info.width)];
        }
      }
      end = std::chrono::system_clock::now();
      long elapsed_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
      ROS_INFO("r: %d, x = %d, y = %d. Elapsed time: %lu", r, r_transform_x, r_transform_y, elapsed_milliseconds);
    }

    // ROS_INFO("MIN: x = %d, y = %d, MAX: x = %d, y = %d", min_X_coord, min_Y_coord, max_X_coord, max_Y_coord);
    pub_.publish(united_map);
    ros::spinOnce();
  }
  return 0;
}