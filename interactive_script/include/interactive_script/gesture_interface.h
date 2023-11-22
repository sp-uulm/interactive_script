#ifndef GESTURE_INTERFACE_H
#define GESTURE_INTERFACE_H

#include "gesture_msgs/msg/gesture.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <optional>

class GestureInterface {
private:
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<rclcpp::Subscription<gesture_msgs::msg::Gesture>> gesture_sub;
    std::shared_ptr<gesture_msgs::msg::Gesture> last_msg = nullptr; 

public:
    static constexpr auto GESTURE_TOPIC = "gesture";

    GestureInterface(const std::shared_ptr<rclcpp::Node>& node);
    
    void clear();
    
    std::optional<geometry_msgs::msg::Pose> received(const std::string& name);
};

#endif // GESTURE_INTERFACE_H
