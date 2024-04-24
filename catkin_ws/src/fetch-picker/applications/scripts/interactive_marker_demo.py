#!/usr/bin/env python

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

def handle_viz_input(input):
    if input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo(input.marker_name + ' was clicked.')
    else:
        rospy.loginfo(f'Cannot handle this InteractiveMarker event: {input.event_type}')

def main():
    rospy.init_node('interactive_marker_demo')

    # Create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("simple_marker")

    # Create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "my_marker"
    int_marker.description = "Simple Click Control"
    int_marker.pose.position.x = 1
    int_marker.pose.orientation.w = 1

    # Create a teal cube marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    # Create an InteractiveMarkerControl
    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True
    button_control.markers.append(box_marker)
    int_marker.controls.append(button_control)

    # Add the interactive marker to our collection &
    # tell the server to call this function when feedback is received for this marker
    server.insert(int_marker, handle_viz_input)

    # 'commit' changes and send to all clients
    server.applyChanges()

    # Keep the node alive
    rospy.spin()

if __name__ == '__main__':
    main()