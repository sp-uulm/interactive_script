#ifndef TF_INTERFACE_H
#define TF_INTERFACE_H

#include <quad_common_utils/geometry_msg_helper.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <optional>
#include <string>

class TfInterface {
private:
    static constexpr auto WORLD_FRAME = "world";

    ros::NodeHandle nh;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

public:
    TfInterface() : tf_listener(tf_buffer) {
    }

    std::optional<geometry_msgs::Pose> get_pose(const std::string& object);
};

#endif // TF_INTERFACE_H
