import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

TOPIC_NAME = '/bot_name/camera_node/image/compressed'


class DuckiebotNode(DTROS):
    def __init__(self, node_name, result_topic_name):
        super(DuckiebotNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self.bridge = CvBridge()

        self.sub = rospy.Subscriber(TOPIC_NAME, CompressedImage, self.callback, queue_size=1)

        self.pub = rospy.Publisher(result_topic_name, CompressedImage, queue_size=1)

    def callback(self, msg):
        print(f'received message with type ${type(msg)}')

