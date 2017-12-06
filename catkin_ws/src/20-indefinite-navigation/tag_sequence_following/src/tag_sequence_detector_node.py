#!/usr/bin/env python
import rospy
import tf.transformations as tr
from duckietown_msgs.msg import AprilTagDetectionArray, TagAction
from duckietown_msgs.srv import SetTagsSequence, GetCurrentAction
from std_msgs.msg import Bool
from time import time

__author__ = "Xomak"


class TagSequenceDetectorNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.tag_live_time = rospy.get_param('~tag_live_time',)

        self.tags_topic = rospy.Subscriber('~tags', AprilTagDetectionArray, self.on_tag_detections)
        self.tag_available = rospy.Publisher('~tag_available', Bool)
        self.tags_passed = rospy.Publisher('~tags_passed', TagAction)
        self.sequence_finished = rospy.Publisher('~sequence_finished', Bool)

        self.current_sequence = None
        self.current_index = None

        self.last_detected_tag_id = None
        self.last_detection_time = 0

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

        # TODO: May be we should implement this using time from messages or ROS Time
        if self.last_detected_tag_id is not None and time() - self.last_detection_time > self.tag_live_time:
            self.last_detected_tag_id = None

        if self.last_detected_tag_id is not None:
            if self.current_sequence is not None:
                current_action = self.current_sequence[self.current_index]
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
                    self.tags_passed.publish(TagAction(self.last_detected_tag_id, current_action.action))
                    rospy.loginfo("[%s] Tag %s -> %s" %
                                  (self.node_name, self.last_detected_tag_id, current_action.action))
                    response['action'] = current_action.action
                    response['ok'] = True
                else:
                    response['not_presented'] = True
                    self.current_index = 0
                    rospy.logwarn("[%s] Not presented: tag %s was not found" %
                                  (self.node_name, self.last_detected_tag_id))

                if self.current_index >= (len(self.current_sequence) - 1):
                    self.current_sequence = None
                    self.sequence_finished.publish(Bool(True))
                self.current_index += 1

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
            self.last_detection_time = time()
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
