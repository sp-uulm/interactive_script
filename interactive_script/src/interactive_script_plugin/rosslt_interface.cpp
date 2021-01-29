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

    message.get_data().type = visualization_msgs::msg::Marker::SPHERE;

    message.get_data().scale.x = 0.25;
    message.get_data().scale.y = 0.25;
    message.get_data().scale.z = 0.25;

    message.get_data().color.r = 1.0;
    message.get_data().color.g = 1.0;
    message.get_data().color.b = 1.0;
    message.get_data().color.a = 1.0;
    message.get_data().id = n_points++;

    std_msgs::msg::Header header;
    header.stamp = node->now();
    header.frame_id = "world";

    SET_FIELD(message, header, header);

    marker_pub->publish(static_cast<rosslt_msgs::msg::MarkerTracked>(message));
}

void RossltInterface::addLine(double x, double y, double z, double x2, double y2, double z2, Color color, double width) {
}

Tracked<geometry_msgs::Quaternion> quaternion(const Tracked<double>& roll, const Tracked<double>& pitch, const Tracked<double>& yaw) {
    auto halfYaw = yaw * 0.5;
    auto halfPitch = pitch * 0.5;
    auto halfRoll = roll * 0.5;
    auto cosYaw = cos(halfYaw);
    auto sinYaw = sin(halfYaw);
    auto cosPitch = cos(halfPitch);
    auto sinPitch = sin(halfPitch);
    auto cosRoll = cos(halfRoll);
    auto sinRoll = sin(halfRoll);

    Tracked<geometry_msgs::Quaternion> result;
    SET_FIELD(result, x, sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw);
    SET_FIELD(result, y, cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw);
    SET_FIELD(result, z, cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw);
    SET_FIELD(result, w, cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
    return result;
}

void RossltInterface::addPose(Tracked<double> x, Tracked<double> y, Tracked<double> z, Tracked<double> psi) {
    Tracked<visualization_msgs::msg::Marker> message;

    auto pose = GET_FIELD(message, pose);
    auto position = GET_FIELD(pose, position);

    SET_FIELD(position, x, x);
    SET_FIELD(position, y, y);
    SET_FIELD(position, z, z);

    SET_FIELD(pose, position, position);
    SET_FIELD(pose, orientation, quaternion(0.0, 0.0, psi));

    SET_FIELD(message, pose, pose);

    message.get_data().type = visualization_msgs::msg::Marker::SPHERE;

    message.get_data().scale.x = 0.25;
    message.get_data().scale.y = 0.25;
    message.get_data().scale.z = 0.25;

    message.get_data().color.r = 1.0;
    message.get_data().color.g = 1.0;
    message.get_data().color.b = 1.0;
    message.get_data().color.a = 1.0;
    message.get_data().id = n_poses++;

    std_msgs::msg::Header header;
    header.stamp = node->now();
    header.frame_id = "world";

    SET_FIELD(message, header, header);

    marker_pub->publish(static_cast<rosslt_msgs::msg::MarkerTracked>(message));
}
