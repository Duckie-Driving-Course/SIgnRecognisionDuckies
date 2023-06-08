from ...base_package.src.duckiebot_connection import DuckiebotNode
from .apriltag_detector import AprilTagger
import rospy


class AprilTaggerNode(DuckiebotNode):
    def __init__(self, node_name, result_topic_name):
        super(AprilTaggerNode, self).__init__(node_name, result_topic_name)
        self.aprilTagger = AprilTagger()

    def callback(self, msg):
        print(f'received message with type ${type(msg)}')

        converted_img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')  # CV2 Image

        processed_img = self.aprilTagger.process_image(converted_img)

        compressed_result_img = self.bridge.cv2_to_compressed_imgmsg(processed_img)

        self.pub.publish(compressed_result_img)


if __name__ == '__main__':
    node = AprilTaggerNode(node_name='april_tagger_connection', result_topic_name="tagged_images")
    rospy.spin()
