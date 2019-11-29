#include <interactive_script/quad_interface.h>

void QuadcopterInterface::send_new_waypoint(const geometry_msgs::msg::Pose &target) {

    Goal goal;
    goal.target_pose.pose = target;
    goal.target_pose.header.frame_id = WORLD_FRAME;

    current_target = target;

    pose_client->async_send_goal(goal);
}

geometry_msgs::msg::Pose QuadcopterInterface::get_current_pose() {
    return current_position;
}

bool QuadcopterInterface::is_at_target(double tolerance) {
    return fabs(current_target.position.x - current_position.position.x) < tolerance
        && fabs(current_target.position.y - current_position.position.y) < tolerance
        && fabs(current_target.position.z - current_position.position.z) < tolerance;
}

void QuadcopterInterface::on_pose_feedback(const GoalHandle&, const Feedback& feedback) {
    // feedback callback -> update the current_position and resume the user script
    current_position = feedback->current_pose.pose;
}

void QuadcopterInterface::on_result_feedback(const GoalHandle&, const Result& result) {
    // finish callback -> make sure any wait() command in the user script terminates
    // by updating the current_position to the target position.
    current_position = current_target;
}
