#include <interactive_script/quad_interface.h>

void QuadcopterInterface::send_new_waypoint(const geometry_msgs::msg::Pose &target) {

    Pose::Goal goal;
    goal.target_pose.pose = target;
    goal.target_pose.header.frame_id = WORLD_FRAME;

    current_target = target;

    using namespace std::placeholders;
    rclcpp_action::Client<Pose>::SendGoalOptions options;
    options.feedback_callback = std::bind(&QuadcopterInterface::on_pose_feedback, this, _1, _2);
    options.result_callback = std::bind(&QuadcopterInterface::on_result_feedback, this, _1);

    pose_client->async_send_goal(goal, options);
}

geometry_msgs::msg::Pose QuadcopterInterface::get_current_pose() {
    return current_position;
}

bool QuadcopterInterface::is_at_target(double tolerance) {
    return fabs(current_target.position.x - current_position.position.x) < tolerance
        && fabs(current_target.position.y - current_position.position.y) < tolerance
        && fabs(current_target.position.z - current_position.position.z) < tolerance;
}

void QuadcopterInterface::on_pose_feedback(GoalHandle, Feedback feedback) {
    // feedback callback -> update the current_position and resume the user script
    current_position = feedback->current_pose.pose;
}

void QuadcopterInterface::on_result_feedback(Result) {
    // finish callback -> make sure any wait() command in the user script terminates
    // by updating the current_position to the target position.
    current_position = current_target;
}
