#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include "string"

class SubscribeTf {
public:
//    size_t convert_to_index(size_t i, size_t column) {
//        return (initial_map_.info.width * i) + column;
//    }
//
//    size_t convert_to_gmap_index(size_t i, size_t column) {
//        int diff_x =
//                static_cast<int>((gmap_.info.origin.position.x - initial_map_.info.origin.position.x) /
//                                 gmap_.info.resolution);
//        int diff_y =
//                static_cast<int>((gmap_.info.origin.position.y - initial_map_.info.origin.position.y) /
//                                 gmap_.info.resolution);
//        return (gmap_.info.width * (i - diff_y)) + column - diff_x;
//    }

//    void callbackGmap(const nav_msgs::OccupancyGrid &gmap) {
//
//
//        ROS_INFO("Started GMap processing");
//
//        gmap_.header = gmap.header;
//        gmap_.info = gmap.info;
//        gmap_.data = gmap.data;
//
//        size_t gmap_width = gmap.info.width;
//        size_t gmap_height = gmap.info.height;
//
//        ROS_INFO("Gmap width: %zu", gmap_width);
//        ROS_INFO("Gmap height: %zu", gmap_height);
//        ROS_INFO("Gmap origin: %f %f", gmap.info.origin.position.x, gmap.info.origin.position.y);
//
//        ROS_INFO("Started initial map processing");
//
//        size_t initial_map_width = initial_map_.info.width;
//        size_t initial_map_height = initial_map_.info.height;
//
//        ROS_INFO("Initial map width: %zu", initial_map_width);
//        ROS_INFO("Initial map height: %zu", initial_map_height);
//        ROS_INFO("Initial map origin: %f %f", initial_map_.info.origin.position.x, initial_map_.info.origin.position.y);
//
//        ROS_INFO("Diff x: %d", static_cast<int>((gmap_.info.origin.position.x - initial_map_.info.origin.position.x) /
//                                                gmap_.info.resolution));
//        ROS_INFO("Diff y: %d", static_cast<int>((gmap_.info.origin.position.y - initial_map_.info.origin.position.y) /
//                                                gmap_.info.resolution));
//
//        for (size_t i = 0; i < initial_map_width; ++i) {
//            for (size_t j = 0; j < initial_map_height; ++j) {
//                size_t map_index = convert_to_index(i, j);
//                size_t gmap_index = convert_to_gmap_index(i, j);
//                if (gmap_index < gmap.data.size() && gmap_.data[gmap_index] != -1 &&
//                    (initial_map_.data[map_index] != gmap_.data[gmap_index])) {
//                    initial_map_.data[map_index] = gmap_.data[gmap_index];
//                }
//            }
//        }
//
//        pub_.publish(initial_map_);
//        ROS_INFO("Finished inputMap processing");
//    }

private:
    ros::ServiceServer service_;

};



inline size_t index(size_t i, size_t column, uint32_t width) {
    return (width * i) + column;
}

class TransformWithSizes{
public:
    TransformWithSizes(){
        transform = new geometry_msgs::TransformStamped();
        width = 0;
        height = 0;
    }

    geometry_msgs::TransformStamped *getTransform() const {
        return transform;
    }

    void setTransformX(geometry_msgs::TransformStamped transform) {
        TransformWithSizes::transform->transform.translation.x = transform.transform.translation.x;
    }

    void setTransformY(geometry_msgs::TransformStamped transform) {
        TransformWithSizes::transform->transform.translation.y = transform.transform.translation.y;
    }

    int32_t getWidth() const {
        return width;
    }

    void setWidth(int32_t width) {
        TransformWithSizes::width = width;
    }

    int32_t getHeight() const {
        return height;
    }

    void setHeight(int32_t height) {
        TransformWithSizes::height = height;
    }
private:
    geometry_msgs::TransformStamped *transform;
    int32_t width;
    int32_t height;
};

void initialize_united_map(ros::NodeHandle &n_, int robots_number, nav_msgs::OccupancyGrid &united_map,
                           std::vector<geometry_msgs::TransformStamped> &savedTransforms, TransformWithSizes &min_transform_sizes) {
    nav_msgs::GetMap srv;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer); // listener to perform proper transforms
    geometry_msgs::TransformStamped transformStamped; // object to store listened transforms
    ros::ServiceClient client = n_.serviceClient<nav_msgs::GetMap>(
            "/turtlebot1/dynamic_map"); // Service to retrieve map from hector_mapping
    client.call(srv);
    nav_msgs::OccupancyGrid retrieved_map = srv.response.map;
    united_map.header.frame_id = "world";
    united_map.info.resolution = retrieved_map.info.resolution;
    united_map.info.origin.orientation = retrieved_map.info.origin.orientation;
    int32_t min_X_coord = 0; // Minimum X coordinate of cell (0.0) across all the maps relatively to 'world' frame [in cells]
    int32_t max_X_coord = 0;
    int32_t min_Y_coord = 0;
    int32_t max_Y_coord = 0;
    for (int i = 1; i < robots_number + 1; i++) {
        client = n_.serviceClient<nav_msgs::GetMap>(
                "/turtlebot" + std::__cxx11::to_string(i) +
                "/dynamic_map"); // Service to retrieve map from hector_mapping
        client.call(srv);
        retrieved_map = srv.response.map;
        bool transformed = false;
        while (!transformed) {
            try {
                transformStamped = tfBuffer.lookupTransform("world", "robot" + std::__cxx11::to_string(i) + "_tf/map",
                                                            ros::Time(0), ros::Duration(1.0));
                transformed = true;
                ROS_INFO("Transform x: %f, Transform y: %f", transformStamped.transform.translation.x,
                         transformStamped.transform.translation.y);
                savedTransforms.emplace_back(transformStamped);
                int32_t temp =
                        (int32_t) (round(transformStamped.transform.translation.x / united_map.info.resolution)) -
                        retrieved_map.info.width / 2;
                if (min_X_coord > temp) {
                    min_X_coord = temp;
                    min_transform_sizes.setTransformX(transformStamped);
                    min_transform_sizes.setWidth(retrieved_map.info.width);
                }

                temp = (int32_t) (round(transformStamped.transform.translation.x / united_map.info.resolution)) +
                       retrieved_map.info.width / 2;
                if (max_X_coord < temp) max_X_coord = temp;

                temp = (int32_t) (round(transformStamped.transform.translation.y / united_map.info.resolution)) -
                       retrieved_map.info.height / 2;
                if (min_Y_coord > temp) {
                    min_Y_coord = temp;
                    min_transform_sizes.setTransformY(transformStamped);
                    min_transform_sizes.setHeight(retrieved_map.info.height);
                }

                temp = (int32_t) (round(transformStamped.transform.translation.y / united_map.info.resolution)) +
                       retrieved_map.info.height / 2;
                if (max_Y_coord < temp) max_Y_coord = temp;
            }
            catch (tf2::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
                ros::Duration(0.1).sleep();
                continue;
            }
        }
    }
    ROS_INFO("MIN: x = %d, y = %d, MAX: x = %d, y = %d", min_X_coord, min_Y_coord, max_X_coord, max_Y_coord);
    united_map.info.width = (uint32_t) (abs(min_X_coord) + max_X_coord);
    united_map.info.height = (uint32_t) (abs(min_Y_coord) + max_Y_coord);
    united_map.info.origin.position.x = min_X_coord * united_map.info.resolution;
    united_map.info.origin.position.y = min_Y_coord * united_map.info.resolution;
};

// Node is based on the assumption that all the merged maps have same resolution and orientation

int main(int argc, char **argv) {
    //Initiate ROS and needed vars
    ros::init(argc, argv, "maps_union_node");
    ros::NodeHandle n_;
    ros::Publisher pub_ = n_.advertise<nav_msgs::OccupancyGrid>("/united_map", 1);
    ros::ServiceClient client; // client to connect to various services
    nav_msgs::GetMap srv; // srv object to be called
    nav_msgs::OccupancyGrid retrieved_map; // object to store retrieved maps
    nav_msgs::OccupancyGrid united_map; // object to be published
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer); // listener to perform proper transforms
    geometry_msgs::TransformStamped transform; // object to store listened transforms
    int robots_number = 0; // count of Turtlebots
    TransformWithSizes min_transform_sizes;
    std::vector<geometry_msgs::TransformStamped> savedTransforms(
            1); // robots transforms relatively to world frame storage. First cell is empty
    // get robots number
    if (n_.getParam("robots_number", robots_number)) {
        ROS_INFO("Got param robots_number: %d", robots_number);
    } else {
        ROS_ERROR("Failed to get param 'robots_number'. Setting to default value = 2.");
        robots_number = 2;
    }
    // united map initialization
    initialize_united_map(n_, robots_number, united_map, savedTransforms, min_transform_sizes);

    ROS_INFO("United map params{ height: %d, width: %d, origin_x: %f, origin_y: %f", united_map.info.width,
             united_map.info.height, united_map.info.origin.position.x, united_map.info.origin.position.y);
    united_map.data = std::vector<signed char>(united_map.info.width * united_map.info.height, -1);
    // main node cycle
    while (ros::ok()) {
        united_map.header.seq++;
        united_map.header.stamp = ros::Time::now();
        for (int r = 1; r < robots_number + 1; r++) {
            client = n_.serviceClient<nav_msgs::GetMap>(
                    "/turtlebot" + std::__cxx11::to_string(r) +
                    "/dynamic_map"); // Service to retrieve map from hector_mapping
            client.call(srv);
            retrieved_map = srv.response.map;
            auto r_transform_x = (int) round(
                    savedTransforms[r].transform.translation.x / united_map.info.resolution);
            auto r_transform_y = (int) round(
                    savedTransforms[r].transform.translation.y / united_map.info.resolution);
            ROS_INFO("r: %d, x = %d, y = %d", r, r_transform_x, r_transform_y);
            for (size_t i = 0; i < retrieved_map.info.height; ++i) {
                for (size_t j = 0; j < retrieved_map.info.width; ++j) {
                    auto t1 = (int)round(min_transform_sizes.getTransform()->transform.translation.y / united_map.info.resolution);
                    auto t2 = (int)round(min_transform_sizes.getTransform()->transform.translation.x / united_map.info.resolution);
                    signed char & current_cell = united_map.data[index(
                            min_transform_sizes.getHeight()/2 - retrieved_map.info.height/2 - t1 +  r_transform_y + i,
                            min_transform_sizes.getWidth()/2 - retrieved_map.info.width/2 - t2 + r_transform_x + j,
                            united_map.info.width)];
                    if (current_cell == -1) current_cell = retrieved_map.data[index(i, j, retrieved_map.info.width)];
                }
            }
        }
        //ROS_INFO("MIN: x = %d, y = %d, MAX: x = %d, y = %d", min_X_coord, min_Y_coord, max_X_coord, max_Y_coord);
        pub_.publish(united_map);
        ros::spinOnce();
    }
    return 0;
}