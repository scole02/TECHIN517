#!/usr/bin/env python
import rospy
from moveit_python import PlanningSceneInterface
from gazebo_msgs.msg import ModelStates
import geometry_msgs.msg

class GazeboObjectImporter:
    def __init__(self):
        rospy.init_node('gazebo_object_importer', anonymous=True)
        
        # Initialize the PlanningSceneInterface
        self.planning_scene = PlanningSceneInterface("map")
        rospy.sleep(1)  # Allow the PlanningScene to initialize
        
        # Clear any existing objects in the scene at startup
        self.planning_scene.clear()

        # Subscribe to the Gazebo model states
        self.model_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

    def model_states_callback(self, data):
        # Attempt to find 'table1' and 'demo_cube' in the list of model names
        try:
            table_index = data.name.index('table1')
            cube_index = data.name.index('demo_cube')

            # Get the poses from Gazebo model states
            table_pose = data.pose[table_index]
            cube_pose = data.pose[cube_index]

            # Update the table in the planning scene
            self.planning_scene.addBox("table1", 0.91300, 0.91300, 0.04, table_pose.position.x, table_pose.position.y, table_pose.position.z + 0.78)

            # Update the cube in the planning scene
            self.planning_scene.addBox("demo_cube", 0.06, 0.06, 0.06, cube_pose.position.x, cube_pose.position.y, cube_pose.position.z + 0.03)

        except ValueError:
            rospy.loginfo("Table or cube not found in Gazebo model states.")

    def clear_scene(self):
        # Clear the planning scene
        self.planning_scene.clear()

    def run(self):
        rospy.on_shutdown(self.clear_scene)  # Clear the scene on shutdown
        rospy.spin()

if __name__ == '__main__':
    importer = GazeboObjectImporter()
    importer.run()
