#ifndef MARKER_INTERFACE_H
#define MARKER_INTERFACE_H

#include <interactive_markers/interactive_marker_server.h>
#include <quad_common_utils/geometry_msg_helper.h>
#include <ros/ros.h>
#include <string>
#include <functional>
#include <chrono>

class MarkerInterface {
private:
    static constexpr auto WORLD_FRAME = "world";

    ros::NodeHandle nh;
    interactive_markers::InteractiveMarkerServer server;
    int n = 0;

public:
    enum class Color {
        WHITE, RED
    };

    MarkerInterface() : server("simple_marker") {
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
