#!/usr/bin/env python
import rclpy
from rclpy.qos import QoSProfile
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
from dagu_car.dagu_wheels_driver import DaguWheelsDriver

class WheelsDriverNode(rclpy.Node):
    def __init__(self):
        super().__init__('wheels_driver_node')
        self.node_name = self.get_name()
        #rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.estop=False

        # Setup publishers
        self.driver = DaguWheelsDriver()
        #add publisher for wheels command wih execution time
        self.msg_wheels_cmd = WheelsCmdStamped()
        self.pub_wheels_cmd = self.create_publisher(WheelsCmdStamped, '~wheels_cmd_executed',
                                                    qos_profile=QoSProfile(depth=1))

        # Setup subscribers
        self.control_constant = 1.0
        self.sub_topic = rclpy.create_subscriber(WheelsCmdStamped, "~wheels_cmd", qos_profile=QoSProfile(depth=1))
        self.sub_topic = rclpy.create_subscriber(BoolStamped, "~emergency_stop", qos_profile=QoSProfile(depth=1))
        # self.sub_topic = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1)
        # self.sub_e_stop = rospy.Subscriber("~emergency_stop", BoolStamped, self.cbEStop, queue_size=1)

    def setupParam(self,param_name,default_value):
        pass
        # value = rospy.get_param(param_name,default_value)
        # rospy.set_param(param_name,value) #Write to parameter server for transparancy
        # rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        # return value

    def cbWheelsCmd(self,msg):
        if self.estop:
            self.driver.setWheelsSpeed(left=0.0,right=0.0)
            return
        self.driver.setWheelsSpeed(left=msg.vel_left,right=msg.vel_right)
        # Put the wheel commands in a message and publish
        self.msg_wheels_cmd.header = msg.header
        # Record the time the command was given to the wheels_driver
        self.msg_wheels_cmd.header.stamp = 0# rospy.get_rostime()
        self.msg_wheels_cmd.vel_left = msg.vel_left
        self.msg_wheels_cmd.vel_right = msg.vel_right
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

    def cbEStop(self,msg):
        self.estop=not self.estop
        # if self.estop:
        #     rospy.loginfo("[%s] Emergency Stop Activated")
        # else:
        #     rospy.loginfo("[%s] Emergency Stop Released")

    def destroy_node(self):
        self.on_shutdown()
        super(self, WheelsDriverNode).destroy_node()

    def on_shutdown(self):
        self.driver.setWheelsSpeed(left=0.0,right=0.0)
        #rospy.loginfo("[%s] Shutting down."%(rospy.get_name()))

if __name__ == '__main__':
    # Initialize the node with rospy
    #rospy.init_node('wheels_driver_node', anonymous=False)
    # Create the DaguCar object
    node = WheelsDriverNode()
    # Setup proper shutdown behavior
    # Keep it spinning to keep the node alive
    rclpy.spin()
