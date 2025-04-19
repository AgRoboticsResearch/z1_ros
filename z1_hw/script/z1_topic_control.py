#!/usr/bin/env python3
# filepath: /home/zfei/codes/z1/ws/src/z1_ros/z1_hw/script/z1_topic_control_fixed.py

import rospy
import time
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SwitchController, ListControllers, SwitchControllerRequest

class Z1TopicControl:
    def __init__(self):
        rospy.init_node('z1_topic_control', anonymous=True)
        
        # Initialize controller manager services
        rospy.loginfo("Waiting for controller manager services...")
        try:
            rospy.wait_for_service('/controller_manager/switch_controller', timeout=5.0)
            rospy.wait_for_service('/controller_manager/list_controllers', timeout=5.0)
            self.switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            self.list_controllers = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
            rospy.loginfo("Controller manager services found!")
        except rospy.ROSException as e:
            rospy.logerr(f"Controller manager services not found: {e}")
            raise
            
        # Make sure the position controller is active
        self.ensure_position_controller_running()
        
        # Subscribe to joint commands topic (using JointState for better semantics)
        self.joint_commands_sub = rospy.Subscriber('/joint_commands', JointState, self.joint_commands_callback, queue_size=1)
        
        # Create publishers for joint position controller
        self.joint_position_pub = rospy.Publisher('/z1_joint_group_position_controller/command', Float64MultiArray, queue_size=1)
        
        # Subscribe to joint states to monitor current robot state
        self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback, queue_size=1)
        
        # Store current joint states
        self.current_joint_positions = [0.0] * 6
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        rospy.loginfo("Z1 Topic Control node started")

    def ensure_position_controller_running(self):
        """
        Make sure that the position controller is running and the trajectory controller is stopped
        to avoid resource conflict between controllers
        """
        try:
            controllers = self.list_controllers().controller
            traj_controller_running = False
            pos_controller_running = False
            
            # Check controller states
            for controller in controllers:
                if controller.name == "z1_joint_traj_controller" and controller.state == "running":
                    traj_controller_running = True
                if controller.name == "z1_joint_group_position_controller" and controller.state == "running":
                    pos_controller_running = True
            
            # If position controller is already running, we're done
            if pos_controller_running:
                rospy.loginfo("Position controller is already running")
                return True
                
            # If trajectory controller is running, we need to stop it and start position controller
            if traj_controller_running:
                rospy.loginfo("Stopping trajectory controller and starting position controller...")
                result = self.switch_controller(
                    start_controllers=['z1_joint_group_position_controller'],
                    stop_controllers=['z1_joint_traj_controller'],
                    strictness=SwitchControllerRequest.STRICT,
                    start_asap=True,
                    timeout=5.0
                )
                
                if result.ok:
                    rospy.loginfo("Successfully switched controllers!")
                    # Wait a moment to ensure the controller is fully started
                    rospy.sleep(0.5)
                    return True
                else:
                    rospy.logerr("Failed to switch controllers!")
                    return False
            else:
                # Just start the position controller
                result = self.switch_controller(
                    start_controllers=['z1_joint_group_position_controller'],
                    stop_controllers=[],
                    strictness=SwitchControllerRequest.STRICT,
                    start_asap=True,
                    timeout=5.0
                )
                
                if result.ok:
                    rospy.loginfo("Successfully started position controller!")
                    rospy.sleep(0.5)
                    return True
                else:
                    rospy.logerr("Failed to start position controller!")
                    return False
                
        except Exception as e:
            rospy.logerr(f"Error ensuring controllers: {e}")
            return False

    def joint_commands_callback(self, msg):
        """
        Handle incoming joint commands.
        Expects JointState message with positions for the 6 joints.
        """
        # Extract joint positions in the correct order for the controller
        command_positions = [0.0] * 6
        found_joints = 0
        
        # Map positions from JointState to our joint order
        for i, name in enumerate(msg.name):
            if name in self.joint_names and i < len(msg.position):
                idx = self.joint_names.index(name)
                command_positions[idx] = msg.position[i]
                found_joints += 1
        
        # Check if we have enough joints
        if found_joints < 6:
            rospy.logwarn("Received joint command with insufficient number of recognized joints. Expected 6, got %d", found_joints)
        
        # Forward the commands to position controller
        command_msg = Float64MultiArray()
        command_msg.data = command_positions
        self.joint_position_pub.publish(command_msg)
        
        rospy.loginfo("Sent joint commands: %s", str(command_positions))

    def joint_states_callback(self, msg):
        """
        Store current joint states for monitoring
        """
        # Extract the positions of our 6 joints from the joint_states message
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                self.current_joint_positions[idx] = msg.position[i]

    def run(self):
        """
        Keep the node running to receive commands
        """
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = Z1TopicControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
