import rosbag
from sensor_msgs.msg import PointCloud2

class MockCamera(object): 
    """A MockCamera reads saved point clouds.
    """
    def __init__(self, ptcld_topic):
        """Initializes a MockCamera.
    
        Args:
            ptcld_topic: string, the topic of the point cloud messages.
        """
        self.topic = ptcld_topic

    def read_cloud(self, path) -> PointCloud2:
        """Returns the sensor_msgs/PointCloud2 in the given bag file.
    
        Args:
            path: string, the path to a bag file with a single
            sensor_msgs/PointCloud2 in it.

        Returns: A sensor_msgs/PointCloud2 message, or None if there were no
            PointCloud2 messages in the bag file.
        """
        bag = rosbag.Bag(path)
        topic, msg, t = next(bag.read_messages(topics=[self.topic]))
        print(topic, t)
        
        return msg