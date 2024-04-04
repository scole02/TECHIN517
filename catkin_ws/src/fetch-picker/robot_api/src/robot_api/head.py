import actionlib
from actionlib_msgs.msg import GoalStatus
from actionlib.action_client import CommState
from control_msgs.msg import PointHeadAction, PointHeadGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import trajectory_msgs.msg
from geometry_msgs.msg import PointStamped
import rospy

LOOK_AT_ACTION_NAME = 'head_controller/point_head'  # TODO: Get the name of the look-at action
PAN_TILT_ACTION_NAME = 'head_controller/follow_joint_trajectory'  # TODO: Get the name of the pan/tilt action
PAN_JOINT = ['head_pan_joint']  # TODO: Get the name of the head pan joint
TILT_JOINT = ['head_tilt_joint']  # TODO: Get the name of the head tilt joint
PAN_TILT_TIME = 2.5  # How many seconds it should take to move the head.

class Head(object):
    """
    Head controls Fetch's head.
    
    It provides 2 interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians
    
    For example:
        head = robot_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """
    MIN_PAN = -1.5707963267948966  # Minimum pan angle, in radians.
    MAX_PAN = 1.5707963267948966   # Maximum pan angle, in radians.
    MIN_TILT = -0.7853981633974483 # Minimum tilt angle, in radians.
    MAX_TILT = 0.7853981633974483  # Maximum tilt angle, in radians.

    def __init__(self):
        self.look_at_client = actionlib.SimpleActionClient(LOOK_AT_ACTION_NAME, PointHeadAction)
        self.pan_tilt_client = actionlib.SimpleActionClient(PAN_TILT_ACTION_NAME, FollowJointTrajectoryAction)
        # TODO: Wait for both servers
        self.look_at_client.wait_for_server()
        self.pan_tilt_client.wait_for_server()
        
    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.
        
        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        goal = PointHeadGoal()
        target = PointStamped()
        target.header.frame_id = frame_id
        target.point.x = x
        target.point.y = y
        target.point.z = z
        goal.target = target
        goal.min_duration = rospy.Duration(1.0)
        self.look_at_client.send_goal(goal)
        self.look_at_client.wait_for_result()
        
    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.
        
        Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        # TODO: Check that the pan/tilt angles are within joint limits
        if not (self.MIN_PAN <= pan <= self.MAX_PAN) and (self.MIN_TILT <= tilt <= self.MAX_TILT):
            rospy.logerr("Pan or tilt value out of bounds")
            return
        
        # TODO: Create a trajectory point
        trajectory = trajectory_msgs.msg.JointTrajectory()
        trajectory.joint_names = [PAN_JOINT[0], TILT_JOINT[0]]
        
        # TODO: Set positions of the two joints in the trajectory point
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = [pan, tilt]
        
        # TODO: Set time of the trajectory point
        point.time_from_start = rospy.Duration(PAN_TILT_TIME)
        
        # TODO: Create goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        
        # TODO: Add trajectory point created above to trajectory
        goal.trajectory.points.append(point)
        
        # TODO: Send the goal
        self.pan_tilt_client.send_goal(goal)
        
        # TODO: Wait for result
        self.pan_tilt_client.wait_for_result()