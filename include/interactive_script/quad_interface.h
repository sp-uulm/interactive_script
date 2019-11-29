#ifndef QUAD_INTERFACE_H
#define QUAD_INTERFACE_H

#include <rclcpp_action/client.hpp>
#include <sp_trajectory_msgs/action/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <quad_common_utils/geometry_msg_helper.h>

struct QuadcopterInterface {

    static constexpr auto WORLD_FRAME = "world";
    static constexpr auto POSE_ACTION = "action/pose";

    using Pose = sp_trajectory_msgs::action::Pose;
    using Goal = std::shared_ptr<const Pose::Goal>;
    using Feedback = std::shared_ptr<const Pose::Feedback>;
    using Result = std::shared_ptr<const Pose::Result>;
    using GoalHandle = std::shared_ptr<rclcpp_action::ClientGoalHandle<Pose>>;

    QuadcopterInterface(const std::shared_ptr<rclcpp::Node>& node) : node(node) {
        using namespace std::placeholders;
        rclcpp_action::Client<Pose>::SendGoalOptions options;
        options.feedback_callback = std::bind(&QuadcopterInterface::on_pose_feedback, this, _1);
        options.result_callback = std::bind(&QuadcopterInterface::on_result_feedback, this, _1);
        
        rclcpp_action::Client<Pose>::SharedPtr pose_client = rclcpp_action::create_client<Pose>(node, POSE_ACTION, options);
    }

    std::shared_ptr<rclcpp::Node> node;
        
    rclcpp_action::Client<Pose>::SharedPtr pose_client;
    
    geometry_msgs::msg::Pose current_target;
    geometry_msgs::msg::Pose current_position;

    void send_new_waypoint(const geometry_msgs::msg::Pose& target);
    geometry_msgs::msg::Pose get_current_pose();
    bool is_at_target(double tolerance = 0.1);
    
private:
    void on_pose_feedback(const GoalHandle&, const Feedback& feedback);
    void on_result_feedback(const GoalHandle&, const Result& result);
    
};

#endif // QUAD_INTERFACE_H
