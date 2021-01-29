#ifndef MARKER_INTERFACE_H
#define MARKER_INTERFACE_H

#include <interactive_markers/interactive_marker_server.hpp>
#include "interactive_script/geometry_msg_helper.h"
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <functional>
#include <chrono>

class MarkerInterface {
private:
    static constexpr auto WORLD_FRAME = "world";

    std::shared_ptr<rclcpp::Node> node;
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> server;
    int n_points = 0;
    int n_lines = 0;
    int n_poses = 0;
    int cur_point = 0;
    int cur_line = 0;
    int cur_pose = 0;

public:
    enum class Color {
        WHITE, RED, YELLOW, BLUE
    };

    MarkerInterface(const std::shared_ptr<rclcpp::Node>& node) : node(node) {
        assert(node);
        server = std::make_unique<interactive_markers::InteractiveMarkerServer>("simple_marker", node);    
    }

    void commit();

    void update();

    void addPoint(double x, double y, double z,
                  bool free_x = false, bool free_y = false, bool free_z = false,
                  interactive_markers::InteractiveMarkerServer::FeedbackCallback func = [](const auto&){});
    void addLine(double x, double y, double z, double x2, double y2, double z2, Color color = Color::WHITE, double width = 0.05);
    void addPose(double x, double y, double z, double psi,
                  bool free_x = false, bool free_y = false, bool free_z = false, bool free_psi = false,
                  interactive_markers::InteractiveMarkerServer::FeedbackCallback func = [](const auto&){});

    std::chrono::duration<double> runtime = std::chrono::duration<double>();
};

#endif // MARKER_INTERFACE_H
