#include "interactive_script/tf_interface.h"

std::optional<geometry_msgs::msg::Pose> TfInterface::get_pose(const std::string& object) {
    if (tf_buffer.canTransform(WORLD_FRAME, object, rclcpp::Time(0))) {
        auto origin = geometry_msgs::origin(object);
        return tf_buffer.transform(origin, WORLD_FRAME).pose;
    }

    return std::nullopt;
}
