#!/usr/bin/env python
import rospy
import tf.transformations as tr
from tf2_msgs.msg import TFMessage
from duckietown_msgs.msg import AprilTagDetectionArray, Pose2DStamped, BoolStamped
from duckietown_msgs.srv import GetStopLineForTag
from geometry_msgs.msg import Transform, TransformStamped, Vector3
from std_msgs.msg import Bool
from std_srvs.srv import Empty

__author__ = "Xomak"


class ComplexLocalizationNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.world_frame = "world"
        self.duckiebot_frame = "duckiebot"
        self.last_detected_tag_id = None

        self.line_tag_interval = rospy.get_param('~line_tag_interval', 7)
        self.tf_topic = rospy.Publisher("/tf", TFMessage, queue_size=1, latch=True)

        self.stop_line_topic = rospy.Subscriber('~at_stop_line', BoolStamped, self.on_stop_line)
        self.tags_topic = rospy.Subscriber('~tags', AprilTagDetectionArray, self.on_tag_detections)
        self.odometry_topic = rospy.Subscriber('~pose', Pose2DStamped, self.on_pose)
        self.tag_to_stopline_service = rospy.ServiceProxy('~tag_to_stopline', GetStopLineForTag)
        self.reset_odometry_service = rospy.ServiceProxy('~reset_pose', Empty)

        self.last_tag_detection = None
        self.last_line_detection = None
        self.current_frame_id = None

        rospy.loginfo("[%s] Initialized." % self.node_name)

    def publish_tf(self, x, y, z):
        translation = Vector3()
        translation.x = x
        translation.y = y
        translation.z = z

        transform_stamped = TransformStamped()
        transform_stamped.transform.translation = translation
        transform_stamped.transform.rotation.x = 0
        transform_stamped.transform.rotation.y = 0
        transform_stamped.transform.rotation.z = 0
        transform_stamped.transform.rotation.w = 1
        transform_stamped.header.frame_id = self.current_frame_id
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.child_frame_id = self.duckiebot_frame
        self.tf_topic.publish(TFMessage([transform_stamped]))

    def on_pose(self, msg):
        if self.current_frame_id:
            self.publish_tf(msg.x, msg.y, 0)

    def check_line(self):
        if self.last_line_detection is not None and self.last_tag_detection is not None and \
                        abs(self.last_line_detection.secs - self.last_tag_detection.secs) < self.line_tag_interval:
            stopline_id = self.tag_to_stopline_service(self.last_detected_tag_id).stopline
            if len(stopline_id) > 0:
                self.reset_odometry_service()
                self.current_frame_id = stopline_id
                rospy.loginfo("[{}] Set position to {}".format(self.node_name, stopline_id))
                self.publish_tf(0, 0, 0)

    def on_stop_line(self, msg):
        self.last_line_detection = rospy.Time.now()
        rospy.loginfo("[{}] On stop line".format(self.node_name))
        self.check_line()

    def on_tag_detections(self, msg):
        tags = msg.detections
        tags = [tag for tag in tags if tag.pose.pose.position.x > 0]
        if len(tags) > 0:
            tags_with_angles = []
            for tag in tags:
                orientation = tag.pose.pose.orientation

                euler_angles = tr.euler_from_quaternion([orientation.x,
                                                         orientation.y,
                                                         orientation.z,
                                                         orientation.w])
                tags_with_angles.append((euler_angles[0], tag))

            tags_with_angles.sort(key=lambda x: x[0])
            main_tag = tags_with_angles[0][1]
            self.last_detected_tag_id = main_tag.id
            self.last_tag_detection = rospy.Time.now()
            rospy.loginfo("[{}] Tag detected".format(self.node_name))
            self.check_line()
        else:
            self.last_detected_tag_id = None

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." % self.node_name)


if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('complex_localization_node', anonymous=False)

    # Create the NodeName object
    node = ComplexLocalizationNode()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
