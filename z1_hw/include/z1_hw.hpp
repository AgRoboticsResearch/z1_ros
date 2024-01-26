#ifndef Z1_ROS_CONTROL
#define Z1_ROS_CONTROL

#include "unitree_arm_sdk/unitree_arm.h"

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <sensor_msgs/JointState.h>
#include <ros/publisher.h>
#include <std_srvs/SetBool.h>
#include <ros/console.h>

/**
 * @brief This class wraps the functions of unitree-arm-sdk 
 * for controlling Z1 robots into the ros_control framework.
 */
class Z1HW : public hardware_interface::RobotHW
{
public:
  /**
   * @brief Default constructor
   * 
   * @note Be sure to call the init() method before operation.
   */
  Z1HW(ros::NodeHandle& nh);

  /**
   * @brief Initializes the model informations.
   * 
   */
  void init();
  void dinit();
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;
  
  void gripperCB(const control_msgs::GripperCommandGoalConstPtr& msg);
  void jointCommandsCallback(const sensor_msgs::JointState::ConstPtr& msg);
  bool switchCommandSourceCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface arm_pos_interface;
  hardware_interface::PositionJointInterface grip_pos_interface;
  
  bool has_gripper;
  bool use_topic_commands;
  ros::Publisher joint_command_publisher;
  ros::Subscriber joint_commands_subscriber;
  ros::ServiceServer switch_cmd_source_service; 
  double cmd[6];
  double direct_joints_cmd[6];

  double* pos;
  double* vel;
  double* eff;

  UNITREE_ARM_SDK::UnitreeArm* arm;

  ros::NodeHandle* _nh;

  /* Gripper Action */
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction>* gripper_as;
  control_msgs::GripperCommandFeedback gripper_feedback;
  control_msgs::GripperCommandResult gripper_result;

  double gripper_epsilon = 0.01;
  double gripper_position_cmd{};
  double gripper_effort_cmd{};
};

#endif // Z1_ROS_CONTROL
