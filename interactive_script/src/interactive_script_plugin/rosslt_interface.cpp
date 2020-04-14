#include <interactive_script/rosslt_interface.h>

void RossltInterface::update() {
}

void RossltInterface::commit() {
}

void RossltInterface::addPoint(Tracked<double> x, Tracked<double> y, Tracked<double> z) {
    Tracked<visualization_msgs::msg::Marker> message;

    auto pose = GET_FIELD(message, pose);
    auto position = GET_FIELD(pose, position);

    SET_FIELD(position, x, x);
    SET_FIELD(position, y, y);
    SET_FIELD(position, z, z);

    SET_FIELD(pose, position, position);
    SET_FIELD(message, pose, pose);

    marker_pub->publish(static_cast<rosslt_msgs::msg::MarkerTracked>(message));
}

void RossltInterface::addLine(double x, double y, double z, double x2, double y2, double z2, Color color, double width) {
}

void RossltInterface::addPose(Tracked<double> x, Tracked<double> y, Tracked<double> z, Tracked<double> psi) {
}
