#include <interactive_script/marker_interface.h>

void MarkerInterface::update() {
    ros::spinOnce();
}

void MarkerInterface::commit() {
    auto start = std::chrono::steady_clock::now();
    server.applyChanges();
    server.clear();
    n = 0;
    auto end = std::chrono::steady_clock::now();
    runtime += (end-start);
}

void MarkerInterface::addPoint(double x, double y, double z,
                               bool free_x, bool free_y, bool free_z,
                               interactive_markers::InteractiveMarkerServer::FeedbackCallback func) {

    auto start = std::chrono::steady_clock::now();

    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = WORLD_FRAME;
    int_marker.header.stamp=ros::Time::now();
    int_marker.name = "point_marker_" + std::to_string(n++);
    //int_marker.description = "Simple 1-DOF Control";

    // create a grey box marker
    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::SPHERE;
    box_marker.scale.x = 0.25;
    box_marker.scale.y = 0.25;
    box_marker.scale.z = 0.25;
    box_marker.color.r = 1.0;
    box_marker.color.g = 1.0;
    box_marker.color.b = 1.0;
    box_marker.color.a = 1.0;

    int_marker.pose.position.x = x;
    int_marker.pose.position.y = y;
    int_marker.pose.position.z = z;

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back( box_marker );
    box_control.name = "clicked";
    box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

    // add the control to the interactive marker
    int_marker.controls.push_back( box_control );

    if (free_x) {
        // create a control which will move the box
        // this control does not contain any markers,
        // which will cause RViz to insert two arrows
        visualization_msgs::InteractiveMarkerControl control;
        control.name = "move_x";
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.interaction_mode =
          visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

        // add the control to the interactive marker
        int_marker.controls.push_back(control);
    }

    if (free_y) {
        // create a control which will move the box
        // this control does not contain any markers,
        // which will cause RViz to insert two arrows
        visualization_msgs::InteractiveMarkerControl control;
        control.name = "move_y";
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.interaction_mode =
          visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

        // add the control to the interactive marker
        int_marker.controls.push_back(control);
    }

    if (free_z) {
        // create a control which will move the box
        // this control does not contain any markers,
        // which will cause RViz to insert two arrows
        visualization_msgs::InteractiveMarkerControl control;
        control.name = "move_z";
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.interaction_mode =
          visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

        // add the control to the interactive marker
        int_marker.controls.push_back(control);
    }

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, func);

    auto end = std::chrono::steady_clock::now();
    runtime += (end-start);
}

void MarkerInterface::addLine(double x, double y, double z, double x2, double y2, double z2, Color color, double width) {
    auto start = std::chrono::steady_clock::now();

    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = WORLD_FRAME;
    int_marker.header.stamp=ros::Time::now();
    int_marker.name = "line_marker_" + std::to_string(n++);
    //int_marker.description = "Simple 1-DOF Control";

    // create a grey box marker
    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::ARROW;
    box_marker.scale.x = width;
    box_marker.scale.y = width*1.5;
    box_marker.scale.z = width*5;

    switch (color) {
    case Color::WHITE:
        box_marker.color.r = 1.0;
        box_marker.color.g = 1.0;
        box_marker.color.b = 1.0;
        box_marker.color.a = 1.0;
        break;
    case Color::RED:
        box_marker.color.r = 1.0;
        box_marker.color.g = 0.0;
        box_marker.color.b = 0.0;
        box_marker.color.a = 1.0;
        break;
    }

    geometry_msgs::Point p1;
    p1.x = x;
    p1.y = y;
    p1.z = z;
    box_marker.points.push_back(p1);

    geometry_msgs::Point p2;
    p2.x = x2;
    p2.y = y2;
    p2.z = z2;
    box_marker.points.push_back(p2);

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back( box_marker );

    // add the control to the interactive marker
    int_marker.controls.push_back( box_control );

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, [](const auto& feedback) {
        ROS_INFO_STREAM( feedback->marker_name << " is now at "
              << feedback->pose.position.x << ", " << feedback->pose.position.y
              << ", " << feedback->pose.position.z );
    });

    auto end = std::chrono::steady_clock::now();
    runtime += (end-start);
}

void MarkerInterface::addPose(double x, double y, double z, double psi,
                               bool free_x, bool free_y, bool free_z, bool free_psi,
                               interactive_markers::InteractiveMarkerServer::FeedbackCallback func) {
    auto start = std::chrono::steady_clock::now();

    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = WORLD_FRAME;
    int_marker.header.stamp=ros::Time::now();
    int_marker.name = "pose_marker_" + std::to_string(n++);
    //int_marker.description = "Simple 1-DOF Control";

    int_marker.pose.position.x = x;
    int_marker.pose.position.y = y;
    int_marker.pose.position.z = z;
    int_marker.pose.orientation = geometry_msgs::quaternion();

    // create a grey box marker
    /*visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.scale.x = 0.3;
    box_marker.scale.y = 0.3;
    box_marker.scale.z = 0.3;
    box_marker.color.r = 1.0;
    box_marker.color.g = 1.0;
    box_marker.color.b = 1.0;
    box_marker.color.a = 1.0;
    box_marker.pose.orientation = geometry_msgs::quaternion(0,0,psi);

    visualization_msgs::Marker arrow_marker;
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    arrow_marker.scale.x = 0.3;
    arrow_marker.scale.y = 0.15;
    arrow_marker.scale.z = 0.15;
    arrow_marker.color.r = 1.0;
    arrow_marker.color.g = 1.0;
    arrow_marker.color.b = 1.0;
    arrow_marker.color.a = 1.0;
    arrow_marker.pose.orientation = geometry_msgs::quaternion(0,0,psi);*/

    visualization_msgs::Marker quad_marker;
    quad_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    quad_marker.mesh_resource = "package://trajectory_server/meshes/bebop.dae";
    quad_marker.scale.x = 0.002;
    quad_marker.scale.y = 0.002;
    quad_marker.scale.z = 0.002;
    quad_marker.color.r = 1.0;
    quad_marker.color.g = 1.0;
    quad_marker.color.b = 1.0;
    quad_marker.color.a = 1.0;
    quad_marker.pose.orientation = geometry_msgs::quaternion(0,0,psi);

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    //box_control.markers.push_back( box_marker );
    //box_control.markers.push_back( arrow_marker );
    box_control.markers.push_back( quad_marker );
    box_control.name = "clicked";
    box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    //box_control.orientation = geometry_msgs::quaternion(0,0,psi);

    // add the control to the interactive marker
    int_marker.controls.push_back( box_control );

    if (free_x) {
        // create a control which will move the box
        // this control does not contain any markers,
        // which will cause RViz to insert two arrows
        visualization_msgs::InteractiveMarkerControl control;
        control.name = "move_x";
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.interaction_mode =
          visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

        // add the control to the interactive marker
        int_marker.controls.push_back(control);
    }

    if (free_y) {
        // create a control which will move the box
        // this control does not contain any markers,
        // which will cause RViz to insert two arrows
        visualization_msgs::InteractiveMarkerControl control;
        control.name = "move_y";
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.interaction_mode =
          visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

        // add the control to the interactive marker
        int_marker.controls.push_back(control);
    }

    if (free_z) {
        // create a control which will move the box
        // this control does not contain any markers,
        // which will cause RViz to insert two arrows
        visualization_msgs::InteractiveMarkerControl control;
        control.name = "move_z";
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.interaction_mode =
          visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

        // add the control to the interactive marker
        int_marker.controls.push_back(control);
    }

    if (free_psi) {
        // create a control which will move the box
        // this control does not contain any markers,
        // which will cause RViz to insert two arrows
        visualization_msgs::InteractiveMarkerControl control;
        control.name = "move_psi";
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.interaction_mode =
          visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;

        // add the control to the interactive marker
        int_marker.controls.push_back(control);
    }

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, func);
    auto end = std::chrono::steady_clock::now();
    runtime += (end-start);
}
