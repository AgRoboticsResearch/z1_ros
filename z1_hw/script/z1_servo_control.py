#!/usr/bin/env python3

import time
import rospkg

z1_sdk_path = rospkg.RosPack().get_path("z1_sdk")
import sys

sys.path.append(z1_sdk_path + "/lib")
import z1_arm_interface
import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
import numpy as np


class MoveitServoZ1(object):
    def __init__(self, rate=100):
        rospy.init_node("z1_servo", anonymous=True, log_level=rospy.INFO)
        self.namespace = rospy.get_namespace()
        self.rate = rospy.Rate(rate)
        rospy.loginfo(rospy.get_name() + " Start")
        rospy.loginfo(rospy.get_name() + " Namespace: " + self.namespace)
        self.servo_init()
        self.arm_init()

        self.define_subscribers()
        self.run()

    def servo_init(self):
        self.joint_pos_states = None
        self.joint_vel_states = None
        self.joint_acc_states = None
        self.twist_data = [0, 0, 0, 0, 0, 0]

    def arm_init(self):
        self.has_gripper = rospy.get_param("UnitreeGripperYN", True)
        controller_ip = rospy.get_param("udp_to_controller/controller_ip", "127.0.0.1")
        controller_port = rospy.get_param("udp/port_to_sdk", 8872)
        sdk_own_port = rospy.get_param("udp_to_controller/own_port", 8872)
        self.z1_arm = z1_arm_interface.ArmInterface(hasGripper=True)
        armState = z1_arm_interface.ArmFSMState
        print("start to move")
        self.z1_arm.loopOn()
        print("ready to move")
        self.z1_arm.startTrack(armState.CARTESIAN)

        # self.z1_arm.loopOn()

    def define_subscribers(self):
        # command dependent on the servo configure file
        moveit_servo_command_topic = "/servo_server/delta_twist_cmds"
        rospy.Subscriber(
            moveit_servo_command_topic,
            TwistStamped,
            self.twist_cmd_callback,
        )

        joint_state_command_topic = "/z1_gazebo/joint_states"
        rospy.Subscriber(
            joint_state_command_topic,
            JointState,
            self.joint_states_callback,
        )

    def twist_cmd_callback(self, twist_msg: TwistStamped):
        # twist data in cartisian space
        self.twist_data = [
            twist_msg.twist.angular.x,
            twist_msg.twist.angular.y,
            twist_msg.twist.angular.z,
            twist_msg.twist.linear.x,
            twist_msg.twist.linear.y,
            twist_msg.twist.linear.z,
        ]
        # print("get commands")

    def joint_states_callback(self, joint_state_msg: JointState):
        self.joint_pos_states = joint_state_msg.position
        self.joint_vel_states = joint_state_msg.velocity
        self.joint_acc_states = joint_state_msg.effort

    def wait_for_joint_states(self):
        while self.joint_pos_states == None and not rospy.is_shutdown():
            rospy.loginfo(rospy.get_name() + " Joint states have not been recieved!")
            self.rate.sleep()
        print("joint states obtained")

    def run(self):
        # wait for joint states:
        self.wait_for_joint_states()
        while not rospy.is_shutdown():
            # calculate the differential ik and execute the command
            directions = np.array(self.twist_data + [0])
            # TOOD: scale to the speed limit
            # print("start send")
            self.z1_arm.cartesianCtrlCmd(directions, 0.5, 0.5)
            # print("send command of cartisian")
            self.rate.sleep()


if __name__ == "__main__":
    MoveitServoZ1()
