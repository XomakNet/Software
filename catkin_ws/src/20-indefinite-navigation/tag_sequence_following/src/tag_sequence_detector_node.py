#!/usr/bin/env python
import rospy
import tf.transformations as tr
from duckietown_msgs.msg import AprilTagDetectionArray
from duckietown_msgs.srv import SetTagsSequence, GetCurrentAction
from std_msgs.msg import Bool

__author__ = "Xomak"


class TagSequenceDetectorNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.tags_topic = rospy.Subscriber('~tags', AprilTagDetectionArray, self.on_tag_detections)
        self.tag_available = rospy.Publisher('~tag_available', Bool)

        self.current_sequence = None
        self.current_index = None

        self.last_detected_tag_id = None

        rospy.Service("~set_tags_sequence", SetTagsSequence, self.on_sequence)
        rospy.Service("~get_current_action", GetCurrentAction, self.on_get_action)

        rospy.loginfo("[%s] Initialized." % self.node_name)

    def on_sequence(self, req):
        self.current_sequence = req.sequence
        self.current_index = 0
        return {}

    def find_tag_id_in_sequence(self, tag_id):
        for idx, action in enumerate(self.current_sequence):
            if action.tag_id == tag_id:
                return idx
        return None

    def on_get_action(self, req):

        response = {
            'action': '',
            'incorrect_sequence': False,
            'not_presented': False,
            'no_sequence': False,
            'no_tag': False,
            'ok': False,
        }

        if self.last_detected_tag_id is not None:
            if self.current_sequence is not None:
                current_action = self.current_sequence[self.current_index]
                self.current_index += 1
                if current_action.tag_id != self.last_detected_tag_id:
                    next_idx = self.find_tag_id_in_sequence(self.last_detected_tag_id)
                    if next_idx is not None:
                        response['incorrect_sequence'] = True
                        current_action = self.current_sequence[next_idx]
                        self.current_index = next_idx
                        rospy.logwarn("[%s] Incorrect sequence: tag %s was not expected here" %
                                      (self.node_name, self.last_detected_tag_id))
                    else:
                        current_action = None

                if current_action is not None:
                    response['action'] = current_action.action
                    response['ok'] = True
                else:
                    response['not_presented'] = True
                    rospy.logwarn("[%s] Not presented: tag %s was not found" %
                                  (self.node_name, self.last_detected_tag_id))

                if self.current_index >= len(self.current_sequence):
                    self.current_sequence = None

            else:
                response['no_sequence'] = True
                rospy.logwarn("[%s] No sequence" % self.node_name)
        else:
            response['no_tag'] = True
            rospy.logwarn("[%s] No tag" % self.node_name)

        return response

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
            self.tag_available.publish(Bool(True))
        else:
            self.last_detected_tag_id = None
            self.tag_available.publish(Bool(False))

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." % self.node_name)


if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('tag_sequence_detector_node', anonymous=False)

    # Create the NodeName object
    node = TagSequenceDetectorNode()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
