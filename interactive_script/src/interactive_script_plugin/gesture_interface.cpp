#include <interactive_script/gesture_interface.h>

GestureInterface::GestureInterface(const std::shared_ptr<rclcpp::Node>& node) : node(node) {
    assert(node);
    
    gesture_sub = node->create_subscription<gesture_msgs::msg::Gesture>(GESTURE_TOPIC, 10, [this](const gesture_msgs::msg::Gesture::SharedPtr msg){last_msg = msg;});
}

void GestureInterface::clear() {
    last_msg = nullptr;
}

std::optional<geometry_msgs::msg::Pose> GestureInterface::received(const std::string& name) {
    if (last_msg && last_msg->gesture_type == name) {
        auto p = last_msg->pose;
        clear();
        return p;
    }
        
    return std::nullopt;
}
