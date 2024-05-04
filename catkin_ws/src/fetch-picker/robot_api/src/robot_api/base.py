#! /usr/bin/env python

# TODO: import ????????_msgs.msg
import rospy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from copy import deepcopy
from math import sqrt, pi
from tf.transformations import euler_from_quaternion

class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = robot_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        # TODO: Create publisher
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self._odom_sub = rospy.Subscriber('odom', Odometry, callback=self._odom_callback)
        self.odom = None
        
    def _odom_callback(self, msg):
        self.odom = msg.pose.pose
        
    def calc_distance(point1, point2):
        dx = point1.x - point2.x
        dy = point1.y - point2.y
        dz = point1.z - point2.z
        return sqrt(dx*dx + dy*dy + dz*dz)
    
    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        while not self.odom:
            rospy.sleep(0.1)
        # TODO: record start position, use Python's copy.deepcopy
        start = euler_from_quaternion([self.odom.orientation.x, self.odom.orientation.y, self.odom.orientation.z, self.odom.orientation.w])
        # TODO: What will you do if angular_distance is greater than 2*pi or less than -2*pi?
        if angular_distance > 0: # check if direction is CCW
            dir_constant = 1
        else: dir_constant = -1 # CW
        # while abs(angular_distance) > 2 * pi:
        #     self.turn(2 * pi * dir_constant)
        #     print("doing 360")
        #     angular_distance -= (360 * dir_constant)
        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has rotated the desired amount
        # TODO: Be sure to handle the case where the desired amount is negative!
        # is distance_moved < distance_requested ?
        goal_yaw = (start[2] + angular_distance) % (2*pi)
        while not rospy.is_shutdown():
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            direction = -1 if angular_distance < 0 else 1
            self.move(0, direction * speed)
            rate.sleep()             
            cur_yaw = euler_from_quaternion([self.odom.orientation.x, self.odom.orientation.y, self.odom.orientation.z, self.odom.orientation.w])[2]
            # dist_traveled = Base.calc_angle_traveled(self.odom.orientation, start[2], dir_constant)
            remaining_angle = ((goal_yaw - cur_yaw) + (2 * pi)) % (2 * pi)    
            # print(dist_traveled, remaining_angle)       
            # print(goal_yaw, cur_yaw, remaining_angle)
            if remaining_angle < 0.05 or remaining_angle > (2 * pi - 0.05):
                break

        
    # def calc_angle_traveled(cur : Quaternion, start_yaw : float, direction):
    #     return ((direction * (  - start_yaw )) + (2 * pi)) % (2 * pi)
    
    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        while not self.odom:
            rospy.sleep(0.1)
        # TODO: record start position, use Python's copy.deepcopy
        start = deepcopy(self.odom.position)
        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has traveled the desired distance
        # TODO: Be sure to handle the case where the distance is negative!
        while Base.calc_distance(start, self.odom.position) < abs(distance):
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            rate.sleep()
        self.move(0, 0)
            
    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        # TODO: Create Twist msg
        msg = Twist()
        # TODO: Fill out msg
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        # TODO: Publish msg
        self.pub.publish(msg)

    def stop(self):
        """Stops the mobile base from moving.
        """
        # TODO: Publish 0 velocity
        self.move(0, 0)
        rospy.loginfo('stopping')
