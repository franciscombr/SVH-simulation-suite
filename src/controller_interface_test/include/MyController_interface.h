#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <std_msgs/Float64.h> 

class MyControllerInterface : public hardware_interface::RobotHW
{

public:
    MyControllerInterface(ros::NodeHandle& nh);
    ~MyControllerInterface();
    void init();
    void update(const ros::TimerEvent& e);
    void read();
    void write(ros::Duration elapsed_time);

private:
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

    joint_limits_interface::JointLimits limits;
    joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;
    void state_cb_0(const std_msgs::Float64& );
    void state_cb_1(const std_msgs::Float64& );
    void state_cb_2(const std_msgs::Float64& );
    void state_cb_3(const std_msgs::Float64& );
    void state_cb_4(const std_msgs::Float64& );
    void state_cb_5(const std_msgs::Float64& );
    void state_cb_6(const std_msgs::Float64& );
    void state_cb_7(const std_msgs::Float64& );
    void state_cb_8(const std_msgs::Float64& );
    void state_cb_9(const std_msgs::Float64& );
    void state_cb_10(const std_msgs::Float64& );
    void state_cb_11(const std_msgs::Float64& );
    void state_cb_12(const std_msgs::Float64& );
    void state_cb_13(const std_msgs::Float64& );
    void state_cb_14(const std_msgs::Float64& );
    double joint_position_[15];
    double joint_velocity_[15];
    double joint_effort_[15];
    double joint_position_command_[15];
    double joint_position_temp[15];
    ros::NodeHandle nh_;
    ros::Publisher joint_pub_[15];
    ros::Subscriber state_sub_[15];
    ros::Timer my_control_loop_;
    ros::Duration elapsed_time_;
    double loop_hz_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    std::vector<std::string> pub_names_ {
        "mujoco_ros/shoulder_pan_joint/shoulder_pan_joint_control",
        "mujoco_ros/shoulder_lift_joint/shoulder_lift_joint_control",
        "mujoco_ros/elbow_joint/elbow_joint_control",
        "mujoco_ros/wrist_1_joint/wrist_1_joint_control",
        "mujoco_ros/wrist_2_joint/wrist_2_joint_control",
        "mujoco_ros/wrist_3_joint/wrist_3_joint_control",
        "mujoco_ros/right_hand_Thumb_Opposition/right_hand_Thumb_Opposition_control",
        "mujoco_ros/right_hand_Thumb_Flexion/right_hand_Thumb_Flexion_control",
        "mujoco_ros/right_hand_Index_Finger_Proximal/right_hand_Index_Finger_Proximal_control",
        "mujoco_ros/right_hand_Index_Finger_Distal/right_hand_Index_Finger_Distal_control",
        "mujoco_ros/right_hand_Middle_Finger_Proximal/right_hand_Middle_Finger_Proximal_control",
        "mujoco_ros/right_hand_Middle_Finger_Distal/right_hand_Middle_Finger_Distal_control",
        "mujoco_ros/right_hand_Finger_Spread/right_hand_Finger_Spread_control",
        "mujoco_ros/right_hand_Pinky/right_hand_Pinky_control",
        "mujoco_ros/right_hand_Ring_Finger/right_hand_Ring_Finger_control"
    };
    std::vector<std::string> sub_names_ {
        "mujoco_ros/shoulder_pan_joint/state",
        "mujoco_ros/shoulder_lift_joint/state",
        "mujoco_ros/elbow_joint/state",
        "mujoco_ros/wrist_1_joint/state",
        "mujoco_ros/wrist_2_joint/state",
        "mujoco_ros/wrist_3_joint/state",
        "mujoco_ros/right_hand_Thumb_Opposition/state",
        "mujoco_ros/right_hand_Thumb_Flexion/state",
        "mujoco_ros/right_hand_Index_Finger_Proximal/state",
        "mujoco_ros/right_hand_Index_Finger_Distal/state",
        "mujoco_ros/right_hand_Middle_Finger_Proximal/state",
        "mujoco_ros/right_hand_Middle_Finger_Distal/state",
        "mujoco_ros/right_hand_Finger_Spread/state",
        "mujoco_ros/right_hand_Pinky/state",
        "mujoco_ros/right_hand_Ring_Fingerstate/"
    };

};