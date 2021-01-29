#ifndef ROSSLT_DEMO_INTERFACE_H
#define ROSSLT_DEMO_INTERFACE_H

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "rosslt/trackingnode.h"
#include "rosslt_msgs/msg/pose_tracked.hpp"

#include "luainterpreter.h"
#include "luaparser.h"

#include "interactive_script/geometry_msg_helper.h"
#include <string>
#include <functional>
#include <chrono>

class RossltDemoInterface {
private:
    static constexpr auto WORLD_FRAME = "world";

    std::unique_ptr<LocationManager> location_manager;
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<rclcpp::Publisher<rosslt_msgs::msg::PoseTracked>> pose_publisher;

public:
    using Callback = std::function<void(int, const std::string&)>;

    RossltDemoInterface(const std::shared_ptr<rclcpp::Node>& node) : node(node) {
        using namespace rclcpp;
        location_manager = std::make_unique<LocationManager>(*node);
        pose_publisher = node->create_publisher<rosslt_msgs::msg::PoseTracked>("target_pose", QoS(KeepLast(10)));
    }

    Tracked<double> code_value(const Callback& change_cb, const lua::rt::val& value) {
        if (value.source) {
            int32_t id;

            int offset = 0;
            int length = 0;

            if (auto sv = dynamic_cast<lua::rt::sourceval*>(value.source.get()); sv) {
                offset = sv->location.front().pos;
                length = sv->location.back().pos + sv->location.back().length - sv->location.front().pos;
            } else if (auto binop = dynamic_cast<lua::rt::sourcebinop*>(value.source.get()); binop) {
                offset = binop->op.pos;
                length = binop->op.length;
            } else if (auto unop = dynamic_cast<lua::rt::sourceunop*>(value.source.get()); unop) {
                offset = unop->op.pos;
                length = unop->op.length;
            }

            source_location sl = source_location::current("lua-script", "main", offset, length);

            LocationFunc lf {
                // get
                [val = value](int32_t id) mutable -> std::string {
                    return val.reevaluate().to_string();
                },

                // set
                change_cb
            };

            id = location_manager->create_location(lf, sl);

            Location location {node->get_fully_qualified_name(), id};
            return Tracked<double>(value.def_number(), location);
        } else {
            return Tracked<double>(value.def_number());
        }
    }

    void send_new_waypoint(const Tracked<geometry_msgs::msg::Pose> &target);
    geometry_msgs::msg::Pose get_current_pose();
    bool is_at_target(double tolerance = 0.1);
};

#endif // ROSSLT_DEMO_INTERFACE_H
