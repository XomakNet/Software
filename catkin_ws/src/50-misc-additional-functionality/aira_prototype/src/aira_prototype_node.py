#!/usr/bin/env python
import json

import rospy
from duckietown_msgs.msg import AprilTagDetectionArray, FSMState, TagAction
from duckietown_msgs.srv import SetTagsSequence
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Bool
from aira_prototype.core.contracts import RobotLiabilityFactoryContract, RobotLiabilityContract
from aira_prototype.core.data_publisher import IPFSConnector

from web3 import HTTPProvider
from web3 import Web3
import ipfsapi

__author__ = "Xomak"


class AiraPrototypeNode(object):

    def __init__(self):
        self.node_name = rospy.get_name()

        self.passed_tags_topic = rospy.Subscriber('~tags_passed', TagAction, self.on_tag_passed)
        self.sequence_finished_topic = rospy.Subscriber('~sequence_finished', Bool, self.on_sequence_finished)

        self.web3_rpc_url = rospy.get_param('~web3_rpc_url')
        self.factory_contract_address = rospy.get_param('~factory_contract_address')
        self.machine_address = rospy.get_param('~machine_address')

        self.web3 = Web3(HTTPProvider(self.web3_rpc_url, request_kwargs={'timeout': 60}))
        self.factory_contract = RobotLiabilityFactoryContract(self.web3,
                                                              self.factory_contract_address,
                                                              self.machine_address)
        self.tracking_liabilities = []
        self.active_liability = None
        self.current_log = []

        self.set_tags_sequence = rospy.ServiceProxy('~set_tags_sequence', SetTagsSequence)
        self.init_movement = rospy.ServiceProxy('~init_movement', Empty)
        self.ipfs_client = ipfsapi.connect()
        self.ipfs_connector = IPFSConnector(self.ipfs_client)

        self.new_block_filter = self.web3.eth.filter('latest')
        self.new_block_filter.watch(self.new_block_callback)

        rospy.loginfo("[%s] Initialized." % self.node_name)

    def on_tag_passed(self, msg):
        self.current_log.append(msg.tag_id)

    def on_sequence_finished(self, msg):
        rospy.loginfo("Sequence finished. Publishing log to IPFS...")
        ipfs_hash = self.ipfs_connector.create_result(self.current_log)
        rospy.loginfo("Published: %s. Applying to blockchain." % ipfs_hash)
        self.active_liability.set_result(ipfs_hash)
        rospy.loginfo("Finished.")
        self.active_liability = None

    def execute_liability(self, liability):
        rospy.loginfo("Executing liability %s..." % liability.contract.address)
        self.active_liability = liability
        objective_hash = liability.get_objective()
        objective_json = self.ipfs_connector.get_robot_sequence(objective_hash)
        rospy.loginfo("Objective gathered from IPFS")
        objective = json.loads(objective_json)
        tag_sequence = []
        self.current_log = []

        for goal in objective:
            action = TagAction(int(goal['tag_id']), goal['action'])
            tag_sequence.append(action)

        self.set_tags_sequence(tag_sequence)
        self.init_movement()

    def new_block_callback(self, block):
        new_contracts = self.factory_contract.get_new_contracts(self.machine_address)
        for contract_address in new_contracts:
            rospy.loginfo("New contract: %s" % contract_address)
            self.tracking_liabilities.append(RobotLiabilityContract(self.web3, contract_address, self.machine_address))

        updated_liabilities = []

        for liability in self.tracking_liabilities:
            objective = liability.get_objective()
            required_in_future = True
            if objective is not None:
                rospy.loginfo("Liability %s is waiting to be executed." % liability.contract.address)
                if self.active_liability is None:
                    self.execute_liability(liability)
                    required_in_future = False

            if required_in_future:
                updated_liabilities.append(liability)

        self.tracking_liabilities = updated_liabilities

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." % self.node_name)


if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('aira_prototype_node', anonymous=False)

    # Create the NodeName object
    node = AiraPrototypeNode()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
