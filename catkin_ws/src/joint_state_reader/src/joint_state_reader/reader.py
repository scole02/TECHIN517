#!/usr/bin/env python                                                                                  
                                                                                                       
import rospy         
from sensor_msgs.msg import JointState                                                                                  
                                                                                                       
class JointStateReader(object):                                                                        
    """Listens to /joint_states and provides the latest joint angles.                                  
                                                                                                       
    Usage:                                                                                             
        joint_reader = JointStateReader()                                                              
        rospy.sleep(0.1)                                                                               
        joint_reader.get_joint('shoulder_pan_joint')                                                   
        joint_reader.get_joints(['shoulder_pan_joint', 'shoulder_lift_joint'])                         
    """                                                                                                
    def __init__(self):                                                                                
        self.sub = rospy.Subscriber('joint_states', JointState, self.callback)  
        self.joints = {}        
    
    def callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            # if name not in self.joints.keys():
            self.joints[name] = msg.position[i]                                                                          
                                                                                                       
    def get_joint(self, name):                                                                         
        """Gets the latest joint value.                                                                
                                                                                                       
        Args:                                                                                          
            name: string, the name of the joint whose value we want to read.                           
                                                                                                       
        Returns: the joint value, or None if we do not have a value yet.                               
        """
        if name in self.joints.keys():                                                                                                                                                  
            return self.joints[name]
        else:
            rospy.loginfo(f'Key: {name} is not a valid joint name.\nValid Names are: {self.joints.keys()}')                                                                                      
                                                                                                       
    def get_joints(self, names):                                                                       
        """Gets the latest values for a list of joint names.                    
                                                                                
        Args:                                                                   
            name: list of strings, the names of the joints whose values we want 
                to read.                                                        
                                                                                
        Returns: A list of the joint values. Values may be None if we do not    
            have a value for that joint yet.                                    
        """       
        return [self.joints[name] if name in self.joints.keys() else 0 for name in names]
