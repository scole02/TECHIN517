#! /usr/bin/env python

import robot_api
import rospy
from tf import TransformListener
import tf


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('arm_demo')
    wait_for_time()
    listener = TransformListener()
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('base_link', 'wrist_roll_link', rospy.Time(0))
            print(f'{trans} {rot}')
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rospy.sleep(1)
    

if __name__ == '__main__':
    main()
