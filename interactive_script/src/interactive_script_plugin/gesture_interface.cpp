#include <interactive_script/gesture_interface.h>

GestureInterface::GestureInterface(const std::shared_ptr<rclcpp::Node>& node) : node(node) {
    assert(node);
    
    gesture_sub = node->create_subscription<gesture_msgs::msg::Gesture>(GESTURE_TOPIC, 10,
        [this](const gesture_msgs::msg::Gesture::SharedPtr msg){
            const std::lock_guard<std::mutex> lock(last_message_mutex);
            last_msg = msg;
        });
}

void GestureInterface::clear() {
    const std::lock_guard<std::mutex> lock(last_message_mutex);
    last_msg = nullptr;
}

std::optional<geometry_msgs::msg::Pose> GestureInterface::received(const std::string& name) {
    if (const std::lock_guard<std::mutex> lock(last_message_mutex); last_msg && last_msg->gesture_type == name) {
        auto p = last_msg->pose;
        last_msg = nullptr;
        return p;
    }
        
    return std::nullopt;
}
