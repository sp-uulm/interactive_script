#ifndef QUAD_INTERFACE_H
#define QUAD_INTERFACE_H

#include <actionlib/client/simple_action_client.h>
#include <hector_uav_msgs/PoseAction.h>
#include <ros/ros.h>
#include <quad_common_utils/geometry_msg_helper.h>

struct QuadcopterInterface {

    static constexpr auto WORLD_FRAME = "world";
    static constexpr auto POSE_ACTION = "action/pose";

    ros::NodeHandle nh;
    actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> pose_client {nh, POSE_ACTION};

    geometry_msgs::Pose current_target;
    geometry_msgs::Pose current_position;

    void send_new_waypoint(const geometry_msgs::Pose& target);
    geometry_msgs::Pose get_current_pose();
    bool is_at_target();
};

#endif // QUAD_INTERFACE_H
