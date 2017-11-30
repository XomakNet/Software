#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import BoolStamped, AprilTagDetectionArray, FSMState
from duckietown_msgs.srv import SetTagsSequence, GetCurrentAction
from std_srvs.srv import Empty, EmptyResponse


__author__ = "Xomak"


class TagSequenceControllerNode(object):

    def __init__(self):
        self.node_name = rospy.get_name()

        self.fsm_state_topic = rospy.Subscriber('~fsm_state', FSMState, self.on_fsm_state)
        self.intersection_state = rospy.get_param('~intersection_state')
        self.turn_service_prefix = rospy.get_param('~turn_service_prefix', '~turn_')

        self.get_current_action = rospy.ServiceProxy('~get_current_action', GetCurrentAction)

        rospy.Service("~init_movement", Empty, self.on_init_movement)

        rospy.loginfo("[%s] Initialized." % self.node_name)

    def on_init_movement(self, req):
        self.turn()
        return EmptyResponse()

    def turn(self):
        rospy.loginfo("Requesting action info...")

        action_info = self.get_current_action()
        if action_info.ok:
            rospy.loginfo("Will perform: %s" % action_info.action)
            turn = rospy.ServiceProxy(self.turn_service_prefix + action_info.action, Empty)
            turn()

    def on_fsm_state(self, msg):
        if msg.state == self.intersection_state:
            self.turn()

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." % self.node_name)


if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('tag_sequence_controller_node', anonymous=False)

    # Create the NodeName object
    node = TagSequenceControllerNode()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
