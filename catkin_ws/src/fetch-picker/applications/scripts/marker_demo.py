#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry
from datetime import datetime
from math import sqrt


def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def show_text_in_rviz(marker_publisher, text):
    marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                lifetime=rospy.Duration(1.5),
                pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.06, 0.06, 0.06),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                text=text)
    marker_publisher.publish(marker)


def spawn_odom_path_marker(marker_publisher : rospy.Publisher, path):
    marker = Marker(
                type=Marker.SPHERE_LIST,
                id=0,
                action=Marker.ADD,       
                lifetime=rospy.Duration(10),
                points=path,
                scale=Vector3(0.1, 0.1, 0.1),
                header=Header(frame_id='odom'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                )
    marker_publisher.publish(marker)
    
class NavPath(object):
    def __init__(self, marker_publisher):
        # Odometry.pose.pose.position
        self.ODOM_MARKER_DELAY_SEC = 2
        self._path = [Point(0,0,0)] 
        self.ids = 0 # incremented each time a marker is added to rviz
        self.time_counter_secs = rospy.get_time() +  self.ODOM_MARKER_DELAY_SEC
        self.marker_publisher = marker_publisher

    def calculate_distance(point1, point2):
        dx = point1.x - point2.x
        dy = point1.y - point2.y
        dz = point1.z - point2.z
        return sqrt(dx*dx + dy*dy + dz*dz)
    def callback(self, msg):
        if NavPath.calculate_distance(self._path[-1], msg.pose.pose.position) > 0.2:        
            self._path.append(msg.pose.pose.position)
            spawn_odom_path_marker(self.marker_publisher, self._path)
    
def main():
    rospy.init_node('my_node')
    marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    # rospy.sleep(1)                                                             
    # show_text_in_rviz(marker_publisher, 'Hello, world!')
    # wait_for_time()    
    nav_path = NavPath(marker_publisher)
    rospy.Subscriber('odom', Odometry, nav_path.callback)
    rospy.spin()

if __name__ == '__main__':
    
  main()