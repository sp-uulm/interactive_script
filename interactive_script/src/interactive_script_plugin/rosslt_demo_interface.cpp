#include <interactive_script/rosslt_demo_interface.h>

void RossltDemoInterface::send_new_waypoint(const Tracked<geometry_msgs::msg::Pose> &target) {
    pose_publisher->publish(static_cast<rosslt_msgs::msg::PoseTracked>(target));
}

geometry_msgs::msg::Pose RossltDemoInterface::get_current_pose() {
    // TODO
    return geometry_msgs::origin();
}

bool RossltDemoInterface::is_at_target(double tolerance) {
    // TODO
    return false;
}

