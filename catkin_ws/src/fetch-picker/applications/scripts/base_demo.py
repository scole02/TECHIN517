#! /usr/bin/env python

import math
import robot_api
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

BASE = robot_api.Base()


def wait_for_time():                                                                          
    """Wait for simulated time to begin.
    """                                                                                       
    while rospy.Time().now().to_sec() == 0:                                                   
        pass


def handle_viz_input(input):
    if input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo(input.marker_name + ' was clicked.')
    else:
        rospy.loginfo(f'Cannot handle this InteractiveMarker event: {input.event_type}')

def handle_fwd_mvmt(input):
    if input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo(input.marker_name + ' was clicked.')
        BASE.go_forward(0.5)

def handle_back_mvmt(input):
    if input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo(input.marker_name + ' was clicked.')
        BASE.go_forward(-0.5)


def handle_left_mvmt(input):
    if input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo(input.marker_name + ' was clicked.')
        BASE.turn(0.523599)
        

def handle_right_mvmt(input):
    if input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo(input.marker_name + ' was clicked.')
        BASE.turn(-0.523599)

def print_usage():                                                                            
    print('usage: rosrun applications base_demo.py move 0.1')                                  
    print('       rosrun applications base_demo.py rotate 30')                                 
    
def make_interactive_marker(name, x, y, z):
    # Create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = name
    int_marker.description = name
    int_marker.pose.position.x = x
    int_marker.pose.position.y = y
    int_marker.pose.position.z = z
    int_marker.pose.orientation.w = 1
    
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
    
    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True
    button_control.markers.append(box_marker)
    int_marker.controls.append(button_control)
    return int_marker

def main():
    rospy.init_node('base_demo')
    wait_for_time()
    # argv = rospy.myargv()
    # if len(argv) < 3:
    #     print_usage() 
    #     return
    # command = argv[1]
    # value = float(argv[2])                                                                    

    server = InteractiveMarkerServer("marker_controls")
    fwd_marker = make_interactive_marker("fwd", x=1, y=0, z=0)
    back_marker = make_interactive_marker("back", x=-1, y=0, z=0)
    left_marker = make_interactive_marker("left", x=0, y=1, z=0)
    right_marker = make_interactive_marker("right", x=0, y=-1, z=0)
    
    server.insert(fwd_marker, handle_fwd_mvmt)
    server.insert(left_marker, handle_left_mvmt)
    server.insert(right_marker, handle_right_mvmt)
    server.insert(back_marker, handle_back_mvmt)
    
    server.applyChanges()

    
    # if command == 'move':                                                                     
    #     base.go_forward(value)
    # elif command == 'rotate':                                                                 
    #     base.turn(value * math.pi / 180)                                                      
    # else:
    #     print_usage()
    rospy.spin()

if __name__ == '__main__':
    main()