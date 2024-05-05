import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
import trajectory_msgs.msg
from .arm_joints import ArmJoints
from .moveit_goal_builder import MoveItGoalBuilder
from moveit_msgs.msg import MoveItErrorCodes, MoveGroupAction                                  
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

FOLLOW_JOINT_TRAJ_ACTION_NAME = "arm_controller/follow_joint_trajectory"
MOVE_GROUP_ACTION_NAME = "move_group"
TIME_FROM_START = 5  # How many seconds it should take to set the arm position.
JOINT_NAME=["shoulder_pan_joint",
            "shoulder_lift_joint",
            "upperarm_roll_joint",
            "elbow_flex_joint",
            "forearm_roll_joint",
            "wrist_flex_joint",
            "wrist_roll_joint"]

class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = robot_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        # Create actionlib clients
        self._follow_joint_traj_client = actionlib.SimpleActionClient(FOLLOW_JOINT_TRAJ_ACTION_NAME, FollowJointTrajectoryAction)
        if not self._follow_joint_traj_client.wait_for_server():
            rospy.logerr(f"Could not connect to follow_joint_trajectory action server {FOLLOW_JOINT_TRAJ_ACTION_NAME}")
            rospy.signal_shutdown("Could not connect to follow_joint_trajectory action server")
            return

        
        self._move_group_client = actionlib.SimpleActionClient(MOVE_GROUP_ACTION_NAME, MoveGroupAction)
        if not self._move_group_client.wait_for_server():
            rospy.logerr(f"Could not connect to move_group action server {MOVE_GROUP_ACTION_NAME}")
            rospy.signal_shutdown("Could not connect to move_group action server")
            return
        
    def move_to_joints(self, arm_joints : ArmJoints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # Create a trajectory point
        # Set position of trajectory point
        # Set time of trajectory point
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = arm_joints.values()
        point.time_from_start = rospy.Duration(TIME_FROM_START)

        #Create goal
        #Add joint name to list
        #Add the trajectory point created above to trajectory
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = arm_joints.names()
        goal.trajectory.points.append(point)

        # Send goal
        self._follow_joint_traj_client.send_goal(goal)
        # Wait for result
        self._follow_joint_traj_client.wait_for_result(rospy.Duration(5.0))

    def move_to_pose(self,
                     pose_stamped,
                     allowed_planning_time=10.0,
                     execution_timeout=15.0,
                     group_name='arm',
                     num_planning_attempts=1,
                     plan_only=False,
                     replan=False,
                     replan_attempts=5,
                     tolerance=0.01,
                     orientation_constraint=None):
        """Moves the end-effector to a pose, using motion planning.

        Args:
            pose: geometry_msgs/PoseStamped. The goal pose for the gripper.
            allowed_planning_time: float. The maximum duration to wait for a
                planning result, in seconds.
            execution_timeout: float. The maximum duration to wait for
                an arm motion to execute (or for planning to fail completely),
                in seconds.
            group_name: string. Either 'arm' or 'arm_with_torso'.
            num_planning_attempts: int. The number of times to compute the same
                plan. The shortest path is ultimately used. For random
                planners, this can help get shorter, less weird paths.
            plan_only: bool. If True, then this method does not execute the
                plan on the robot. Useful for determining whether this is
                likely to succeed.
            replan: bool. If True, then if an execution fails (while the arm is
                moving), then come up with a new plan and execute it.
            replan_attempts: int. How many times to replan if the execution
                fails.
            tolerance: float. The goal tolerance, in meters.

        Returns:
            string describing the error if an error occurred, else None.
        """
        goal_builder = MoveItGoalBuilder()
        goal_builder.set_pose_goal(pose_stamped)
        goal_builder.allowed_planning_time = allowed_planning_time
        goal_builder.num_planning_attempts = num_planning_attempts
        goal_builder.plan_only = plan_only
        goal_builder.replan = replan
        goal_builder.replan_attempts = replan_attempts
        goal_builder.tolerance = tolerance
        if orientation_constraint is not None:
            goal_builder.add_path_orientation_constraint(orientation_constraint)
        goal = goal_builder.build()
        # Use execution_timeout for wait_for_result()
        self._move_group_client.send_goal(goal)
        self._move_group_client.wait_for_result(rospy.Duration(execution_timeout))
        result = self._move_group_client.get_result()
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            return f'Error code: {Arm.moveit_error_string(result.error_code.val)}'
        else: return None
        

    def check_pose(self, 
                   pose_stamped,
                   allowed_planning_time=10.0,
                   group_name='arm',
                   tolerance=0.01):
        return self.move_to_pose(
            pose_stamped,
            allowed_planning_time=allowed_planning_time,
            group_name=group_name,
            tolerance=tolerance,
            plan_only=True)
    def compute_ik(self, pose_stamped, timeout=rospy.Duration(5)):
        """Computes inverse kinematics for the given pose.

        Note: if you are interested in returning the IK solutions, we have
            shown how to access them.

        Args:
            pose_stamped: geometry_msgs/PoseStamped.
            timeout: rospy.Duration. How long to wait before giving up on the
                IK solution.

        Returns: True if the inverse kinematics were found, False otherwise.
        """
        request = GetPositionIKRequest()
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.group_name = 'arm'
        request.ik_request.timeout = timeout
        response = self._compute_ik(request)
        error_str = Arm.moveit_error_string(response.error_code.val)
        success = error_str == 'SUCCESS'
        if not success:
            return False
        joint_state = response.solution.joint_state
        for name, position in zip(joint_state.name, joint_state.position):
            if name in ArmJoints.names():
                rospy.loginfo('{}: {}'.format(name, position))
        return True
        
    def cancel_all_goals(self):
        self._follow_joint_traj_client.cancel_all_goals() # Your action client from Lab 7
        self._move_group_client.cancel_all_goals() # From this lab
    
    def moveit_error_string(val: int):
        """Returns a string associated with a MoveItErrorCode.

        Args:
            val: The val field from moveit_msgs/MoveItErrorCodes.msg

        Returns: The string associated with the error value, 'UNKNOWN_ERROR_CODE'
            if the value is invalid.
        """ 

        if val == MoveItErrorCodes.SUCCESS:
            return 'SUCCESS'
        elif val == MoveItErrorCodes.FAILURE:
            return 'FAILURE'
        elif val == MoveItErrorCodes.PLANNING_FAILED:
            return 'PLANNING_FAILED'
        elif val == MoveItErrorCodes.INVALID_MOTION_PLAN:
            return 'INVALID_MOTION_PLAN'
        elif val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
            return 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE'
        elif val == MoveItErrorCodes.CONTROL_FAILED:
            return 'CONTROL_FAILED'
        elif val == MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:
            return 'UNABLE_TO_AQUIRE_SENSOR_DATA'
        elif val == MoveItErrorCodes.TIMED_OUT:
            return 'TIMED_OUT'
        elif val == MoveItErrorCodes.PREEMPTED:
            return 'PREEMPTED'
        elif val == MoveItErrorCodes.START_STATE_IN_COLLISION:
            return 'START_STATE_IN_COLLISION'
        elif val == MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
            return 'START_STATE_VIOLATES_PATH_CONSTRAINTS'
        elif val == MoveItErrorCodes.GOAL_IN_COLLISION:
            return 'GOAL_IN_COLLISION'
        elif val == MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
            return 'GOAL_VIOLATES_PATH_CONSTRAINTS'
        elif val == MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED:
            return 'GOAL_CONSTRAINTS_VIOLATED'
        elif val == MoveItErrorCodes.INVALID_GROUP_NAME:
            return 'INVALID_GROUP_NAME'
        elif val == MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS:
            return 'INVALID_GOAL_CONSTRAINTS'
        elif val == MoveItErrorCodes.INVALID_ROBOT_STATE:
            return 'INVALID_ROBOT_STATE'
        elif val == MoveItErrorCodes.INVALID_LINK_NAME:
            return 'INVALID_LINK_NAME'                                      
        elif val == MoveItErrorCodes.INVALID_OBJECT_NAME:
            return 'INVALID_OBJECT_NAME'
        elif val == MoveItErrorCodes.FRAME_TRANSFORM_FAILURE:
            return 'FRAME_TRANSFORM_FAILURE'
        elif val == MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:
            return 'COLLISION_CHECKING_UNAVAILABLE'
        elif val == MoveItErrorCodes.ROBOT_STATE_STALE:
            return 'ROBOT_STATE_STALE'
        elif val == MoveItErrorCodes.SENSOR_INFO_STALE:
            return 'SENSOR_INFO_STALE'
        elif val == MoveItErrorCodes.NO_IK_SOLUTION:
            return 'NO_IK_SOLUTION'
        else:
            return f'UNKNOWN_ERROR_CODE: {val}'