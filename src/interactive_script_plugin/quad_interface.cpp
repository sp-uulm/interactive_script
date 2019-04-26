#include <interactive_script/quad_interface.h>

void QuadcopterInterface::send_new_waypoint(const geometry_msgs::Pose &target) {

    hector_uav_msgs::PoseGoal goal;
    goal.target_pose.pose = target;
    goal.target_pose.header.frame_id = WORLD_FRAME;

    current_target = target;

    pose_client.sendGoal(goal,
        // finish callback -> make sure any wait() command in the user script terminates
        // by updating the current_position to the target position.
        [this](const auto& state [[gnu::unused]], const auto& result [[gnu::unused]]) {
            current_position = current_target;
        },
        // activation callback -> not needed
        [](){},
        // feedback callback -> update the current_position and resume the user script
        [this](const auto& feedback) {
            current_position = feedback->current_pose.pose;
        }
    );
}

geometry_msgs::Pose QuadcopterInterface::get_current_pose() {
    return current_position;
}

bool QuadcopterInterface::is_at_target() {
    return current_target == current_position;
}
