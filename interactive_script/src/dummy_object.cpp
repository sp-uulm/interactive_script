#include <quad_common_utils/params.h>
#include <quad_common_utils/geometry_msg_helper.h>

#include <interactive_markers/interactive_marker_server.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <ros/ros.h>
#include <string>

PARAM_REQ(OBJECT_NAME, "~name");
PARAM_REQ(COLOR, "~color");
PARAM_REQ(INTERACTIVE_MARKER_TOPIC, "~interactive_marker_topic");
PARAM_DEF(WORLD_FRAME, "~world_frame", "world");
PARAM_DEF(OBJECT_MESH, "~mesh", "package://trajectory_server/meshes/hardhat.dae");
PARAM_DEF(OBJECT_SHAPE, "~shape", visualization_msgs::Marker::MESH_RESOURCE);
PARAM_DEF(POS_X, "~x", 0.0);
PARAM_DEF(POS_Y, "~y", 0.0);
PARAM_DEF(POS_Z, "~z", 0.0);
PARAM_DEF(POS_PSI, "~psi", 0.0);

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "dummy_object");

    ros::NodeHandle nh;
    tf2_ros::TransformBroadcaster br;

    LOAD_PARAM_REQ(OBJECT_NAME, std::string);
    LOAD_PARAM_REQ(INTERACTIVE_MARKER_TOPIC, std::string);
    LOAD_PARAM_REQ(COLOR, std::vector<double>);
    LOAD_PARAM_DEF(WORLD_FRAME, std::string);
    LOAD_PARAM_DEF(OBJECT_MESH, std::string);
    LOAD_PARAM_DEF(OBJECT_SHAPE, int);
    LOAD_PARAM_DEF(POS_X, double);
    LOAD_PARAM_DEF(POS_Y, double);
    LOAD_PARAM_DEF(POS_Z, double);
    LOAD_PARAM_DEF(POS_PSI, double);

    interactive_markers::InteractiveMarkerServer server(INTERACTIVE_MARKER_TOPIC);

    visualization_msgs::InteractiveMarker marker;

    marker.header.frame_id = WORLD_FRAME;
    marker.header.stamp = ros::Time::now();
    marker.name = OBJECT_NAME;
    marker.pose = geometry_msgs::pose(POS_X, POS_Y, POS_Z, POS_PSI);

    visualization_msgs::InteractiveMarkerControl control;

    control.name = "helmet";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
    control.always_visible = true;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    visualization_msgs::Marker m;

    m.scale = geometry_msgs::vec3(1, 1, 1);
    m.mesh_resource = OBJECT_MESH;
    m.type = OBJECT_SHAPE;
    m.header = marker.header;
    m.ns = "object_" + OBJECT_NAME;
    m.id = 0;
    m.action = visualization_msgs::Marker::ADD;
    m.color.r = COLOR[0];
    m.color.g = COLOR[1];
    m.color.b = COLOR[2];
    m.color.a = 1.0;
    m.pose = geometry_msgs::pose(POS_X, POS_Y, POS_Z, POS_PSI + M_PI/2);

    control.markers.push_back(m);
    marker.controls.push_back(control);

    server.insert(marker, [&](const auto& feedback) {
        POS_PSI = geometry_msgs::yaw(feedback->pose.orientation);
        POS_X = feedback->pose.position.x;
        POS_Y = feedback->pose.position.y;
        POS_Z = feedback->pose.position.z;
    });

    server.applyChanges();

    ros::Timer timer = nh.createTimer(ros::Duration(1.0/30), [&](const ros::TimerEvent&) {
        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = WORLD_FRAME;
        transform.child_frame_id = OBJECT_NAME;
        transform.header.stamp = ros::Time::now();
        transform.transform.translation.x = POS_X;
        transform.transform.translation.y = POS_Y;
        transform.transform.translation.z = POS_Z;
        transform.transform.rotation = geometry_msgs::quaternion(0.0, 0.0, POS_PSI);
        br.sendTransform(transform);
    });

    ros::spin();

    return 0;
}
