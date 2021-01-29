#ifndef ROSSLT_INTERFACE_H
#define ROSSLT_INTERFACE_H

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "rosslt/tracked.h"
#include "rosslt/trackingnode.h"
#include "rosslt_msgs/msg/marker_tracked.hpp"

#include "luainterpreter.h"
#include "luaparser.h"

#include "interactive_script/geometry_msg_helper.h"
#include <string>
#include <functional>
#include <chrono>

class RossltInterface {
private:
    static constexpr auto WORLD_FRAME = "world";

    std::unique_ptr<LocationManager> location_manager;
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<rclcpp::Publisher<rosslt_msgs::msg::MarkerTracked>> marker_pub;

    int n_points = 0;
    int n_lines = 0;
    int n_poses = 0;

public:
    using Callback = std::function<void(int, const std::string&)>;

    enum class Color {
        WHITE, RED, YELLOW, BLUE
    };

    RossltInterface(const std::shared_ptr<rclcpp::Node>& node) : node(node) {
        using namespace rclcpp;
        location_manager = std::make_unique<LocationManager>(*node);
        marker_pub = node->create_publisher<rosslt_msgs::msg::MarkerTracked>("tracked_marker", QoS(KeepLast(10)));
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

    void reset() {
        n_poses = n_lines = n_points = 0;
    }

    void commit();

    void update();

    void addPoint(Tracked<double> x, Tracked<double> y, Tracked<double> z);
    void addLine(double x, double y, double z, double x2, double y2, double z2, Color color = Color::WHITE, double width = 0.05);
    void addPose(Tracked<double> x, Tracked<double> y, Tracked<double> z, Tracked<double> psi);
};

#endif // ROSSLT_INTERFACE_H
