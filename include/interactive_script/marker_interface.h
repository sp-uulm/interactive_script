#ifndef MARKER_INTERFACE_H
#define MARKER_INTERFACE_H

#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <string>
#include <functional>

class MarkerInterface {
private:
    ros::NodeHandle nh;
    interactive_markers::InteractiveMarkerServer server;
    int n = 0;

public:
    MarkerInterface() : nh("MarkerInterface"), server("simple_marker") {
    }

    void commit();

    void update();

    void addPoint(double x, double y, double z,
                  bool free_x = false, bool free_y = false, bool free_z = false,
                  interactive_markers::InteractiveMarkerServer::FeedbackCallback func = [](const auto&){});
    void addLine(double x, double y, double z, double x2, double y2, double z2);

};

#endif // MARKER_INTERFACE_H
