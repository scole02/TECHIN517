#!/usr/bin/env python
import rospy
import tf
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from geometry_msgs.msg import PoseStamped, Quaternion
import robot_api

def make_gripper_marker(pose_stamped):
    # Create an Interactive Marker with a gripper and finger meshes representation and 6DOF controls.
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = pose_stamped.header.frame_id
    print(int_marker.header.frame_id)
    int_marker.pose = pose_stamped.pose
    int_marker.scale = 0.2  # Adjust as needed for visibility

    # Create a mesh marker for the gripper base
    gripper_mesh = Marker()
    gripper_mesh.type = Marker.MESH_RESOURCE
    gripper_mesh.mesh_resource = "package://fetch_description/meshes/gripper_link.dae"
    gripper_mesh.color.r, gripper_mesh.color.g, gripper_mesh.color.b, gripper_mesh.color.a = 0.5, 0.5, 0.5, 1.0
    gripper_mesh.scale.x = gripper_mesh.scale.y = gripper_mesh.scale.z = 1.0

    # Adjust position and orientation of the gripper and fingers
    gripper_mesh.pose.position.x = 0.1
    l_finger_mesh = create_finger_mesh("package://fetch_description/meshes/l_gripper_finger_link.STL", {'x': 0.1, 'y': -0.05, 'z': 0.0, 'roll': 0, 'pitch': 0, 'yaw': 0})
    r_finger_mesh = create_finger_mesh("package://fetch_description/meshes/r_gripper_finger_link.STL", {'x': 0.1, 'y': 0.05, 'z': 0.0, 'roll': 0, 'pitch': 0, 'yaw': 0})

    # Gripper control for visualization
    gripper_control = InteractiveMarkerControl()
    gripper_control.interaction_mode = InteractiveMarkerControl.NONE
    gripper_control.always_visible = True
    gripper_control.markers.append(gripper_mesh)
    gripper_control.markers.append(l_finger_mesh)
    gripper_control.markers.append(r_finger_mesh)
    int_marker.controls.append(gripper_control)

    add_6dof_controls(int_marker)
    return int_marker

def create_finger_mesh(mesh_resource, offset):
    # Helper function to create a finger mesh with specified offsets.
    finger_mesh = Marker()
    finger_mesh.type = Marker.MESH_RESOURCE
    finger_mesh.mesh_resource = mesh_resource
    finger_mesh.color.r, finger_mesh.color.g, finger_mesh.color.b, finger_mesh.color.a = 0.5, 0.5, 0.5, 1.0
    finger_mesh.scale.x = finger_mesh.scale.y = finger_mesh.scale.z = 1.0
    finger_mesh.pose.position.x = offset['x']
    finger_mesh.pose.position.y = offset['y']
    finger_mesh.pose.position.z = offset['z']
    finger_mesh.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(offset['roll'], offset['pitch'], offset['yaw']))
    return finger_mesh

def add_6dof_controls(int_marker):
    # Add movement and rotation controls to the interactive marker.
    for axis in ['x', 'y', 'z']:
        move_control = InteractiveMarkerControl()
        move_control.orientation.w = 1
        move_control.orientation.x = 1 if axis == 'x' else 0
        move_control.orientation.y = 1 if axis == 'y' else 0
        move_control.orientation.z = 1 if axis == 'z' else 0
        move_control.name = axis + "_move"
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(move_control)
        
    for axis in ['x', 'y', 'z']:
        rotate_control = InteractiveMarkerControl()
        rotate_control.orientation.w = 1
        rotate_control.orientation.x = 1 if axis == 'x' else 0
        rotate_control.orientation.y = 1 if axis == 'y' else 0
        rotate_control.orientation.z = 1 if axis == 'z' else 0
        rotate_control.name = axis + "_rotate"
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(rotate_control)

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server, offset):
        self.arm = arm
        self.gripper = gripper
        self.im_server = im_server
        self.offset = offset
        self.menu_handler = MenuHandler()
        self.create_menu()

    def create_menu(self):
        # Create a menu for the interactive marker with options for opening and closing the gripper.
        self.menu_handler.insert("Send to Goal Pose", callback=self.menu_callback)
        self.menu_handler.insert("Open Gripper", callback=lambda feedback: self.gripper_command_callback(feedback, 'open'))
        self.menu_handler.insert("Close Gripper", callback=lambda feedback: self.gripper_command_callback(feedback, 'close'))

    def start(self):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose.position.x = 0.5 + self.offset['x']
        pose_stamped.pose.position.y = 0 + self.offset['y']
        pose_stamped.pose.position.z = 1.0 + self.offset['z']
        pose_stamped.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(self.offset['roll'], self.offset['pitch'], self.offset['yaw']))

        gripper_im = make_gripper_marker(pose_stamped)
        self.im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self.menu_handler.apply(self.im_server, gripper_im.name)
        self.im_server.applyChanges()

    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            pose = PoseStamped()
            pose.header.frame_id = feedback.header.frame_id
            pose.pose = feedback.pose
            if self.arm.check_pose(pose):
                self.update_marker_color(feedback.marker_name, 1, 0, 0)
            else:
                self.update_marker_color(feedback.marker_name, 0, 1, 0)

    def menu_callback(self, feedback):
        pose = PoseStamped()
        pose.header.frame_id = feedback.header.frame_id
        pose.pose = feedback.pose
        self.arm.move_to_pose(pose)

    def gripper_command_callback(self, feedback, command):
        if command == 'open':
            self.gripper.open()
        elif command == 'close':
            self.gripper.close()

    def update_marker_color(self, marker_name, r, g, b):
        int_marker = self.im_server.get(marker_name)
        for control in int_marker.controls:
            for marker in control.markers:
                marker.color.r = r
                marker.color.g = g
                marker.color.b = b
        self.im_server.insert(int_marker, self.handle_feedback)
        self.im_server.applyChanges()

def main():
    rospy.init_node('gripper_teleop_node')
    arm = robot_api.Arm()
    gripper = robot_api.Gripper()
    im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)
    offset = {'x': 0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
    teleop = GripperTeleop(arm, gripper, im_server, offset)
    teleop.start()
    rospy.spin()

if __name__ == '__main__':
    main()
