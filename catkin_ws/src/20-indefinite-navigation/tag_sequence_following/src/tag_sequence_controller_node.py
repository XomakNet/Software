#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import AprilTagDetectionArray, FSMState
from duckietown_msgs.srv import SetTagsSequence, GetCurrentAction
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Bool


__author__ = "Xomak"


class TagSequenceControllerNode(object):

    def __init__(self):
        self.node_name = rospy.get_name()

        self.tag_available_topic = rospy.Subscriber('~tag_available', Bool, self.on_tag_available)
        self.fsm_state_topic = rospy.Subscriber('~fsm_state', FSMState, self.on_fsm_state)
        self.intersection_state = rospy.get_param('~intersection_state')
        self.turn_service_prefix = rospy.get_param('~turn_service_prefix', '~turn_')

        self.get_current_action = rospy.ServiceProxy('~get_current_action', GetCurrentAction)

        self.waiting_for_tag = False

        rospy.Service("~init_movement", Empty, self.on_init_movement)

        rospy.loginfo("[%s] Initialized." % self.node_name)

    def on_init_movement(self, req):
        self.turn()
        return {}

    def turn(self):
        rospy.loginfo("Requesting action info...")

        action_info = self.get_current_action()
        if action_info.ok:
            rospy.loginfo("Will perform: %s" % action_info.action)
            if action_info.action != "stop":
                turn = rospy.ServiceProxy(self.turn_service_prefix + action_info.action, Empty)
                turn()
        elif action_info.no_tag:
            rospy.loginfo("No tag available, waiting...")
            self.waiting_for_tag = True

    def on_tag_available(self, msg):
        if self.waiting_for_tag and msg.data:
            rospy.loginfo("Tag available")
            self.waiting_for_tag = False
            self.turn()

    def on_fsm_state(self, msg):
        if msg.state == self.intersection_state:
            rospy.loginfo("FSM changed to intersection state")
            self.turn()
        else:
            self.waiting_for_tag = False

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
