#include "MyController_interface.h"

MyControllerInterface::MyControllerInterface(ros::NodeHandle& nh):nh_(nh){

    // Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot.
    init();
    
    // Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
        
    //Set the frequency of the control loop.
    loop_hz_=50;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        
    //Run the control loop
    my_control_loop_ = nh_.createTimer(update_freq, &MyControllerInterface::update, this);
}

MyControllerInterface::~MyControllerInterface()
{
    
}

void MyControllerInterface::init(){

    // Create joint_state_interface for Joint
    hardware_interface::JointStateHandle jointStateHandleC0("shoulder_pan_joint", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
    joint_state_interface_.registerHandle(jointStateHandleC0);
    hardware_interface::JointStateHandle jointStateHandleC1("shoulder_lift_joint", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
    joint_state_interface_.registerHandle(jointStateHandleC1);
    hardware_interface::JointStateHandle jointStateHandleC2(  "elbow_joint",  &joint_position_[2], &joint_velocity_[2], &joint_effort_[2]);
    hardware_interface::JointStateHandle jointStateHandleC3(  "wrist_1_joint",  &joint_position_[3], &joint_velocity_[3], &joint_effort_[3]);
    hardware_interface::JointStateHandle jointStateHandleC4(  "wrist_2_joint",  &joint_position_[4], &joint_velocity_[4], &joint_effort_[4]);
    hardware_interface::JointStateHandle jointStateHandleC5(  "wrist_3_joint",  &joint_position_[5], &joint_velocity_[5], &joint_effort_[5]);
    hardware_interface::JointStateHandle jointStateHandleC6(  "right_hand_Thumb_Opposition",  &joint_position_[6], &joint_velocity_[6], &joint_effort_[6]);
    hardware_interface::JointStateHandle jointStateHandleC7(  "right_hand_Thumb_Flexion",  &joint_position_[7], &joint_velocity_[7], &joint_effort_[7]);
    hardware_interface::JointStateHandle jointStateHandleC8(  "right_hand_Index_Finger_Proximal",  &joint_position_[8], &joint_velocity_[8], &joint_effort_[8]);
    hardware_interface::JointStateHandle jointStateHandleC9(  "right_hand_Index_Finger_Distal",  &joint_position_[9], &joint_velocity_[9], &joint_effort_[9]);
    hardware_interface::JointStateHandle jointStateHandleC10( "right_hand_Middle_Finger_Proximal", &joint_position_[10], &joint_velocity_[10], &joint_effort_[10]);
    hardware_interface::JointStateHandle jointStateHandleC11( "right_hand_Middle_Finger_Distal", &joint_position_[11], &joint_velocity_[11], &joint_effort_[11]);
    hardware_interface::JointStateHandle jointStateHandleC12( "right_hand_Finger_Spread", &joint_position_[12], &joint_velocity_[12], &joint_effort_[12]);
    hardware_interface::JointStateHandle jointStateHandleC13( "right_hand_Pinky", &joint_position_[13], &joint_velocity_[13], &joint_effort_[13]);
    hardware_interface::JointStateHandle jointStateHandleC14( "right_hand_Ring_Finger", &joint_position_[14], &joint_velocity_[14], &joint_effort_[14]);
    joint_state_interface_.registerHandle(jointStateHandleC2);
    joint_state_interface_.registerHandle(jointStateHandleC3);
    joint_state_interface_.registerHandle(jointStateHandleC4);
    joint_state_interface_.registerHandle(jointStateHandleC5);
    joint_state_interface_.registerHandle(jointStateHandleC6);
    joint_state_interface_.registerHandle(jointStateHandleC7);
    joint_state_interface_.registerHandle(jointStateHandleC8);
    joint_state_interface_.registerHandle(jointStateHandleC9);
    joint_state_interface_.registerHandle(jointStateHandleC10);
    joint_state_interface_.registerHandle(jointStateHandleC11);
    joint_state_interface_.registerHandle(jointStateHandleC12);
    joint_state_interface_.registerHandle(jointStateHandleC13);
    joint_state_interface_.registerHandle(jointStateHandleC14);
    
    // Create position joint interface as JointC accepts position command.
    hardware_interface::JointHandle jointPositionHandleC0(jointStateHandleC0, &joint_position_command_[0]);
    position_joint_interface_.registerHandle(jointPositionHandleC0);
    hardware_interface::JointHandle jointPositionHandleC1(jointStateHandleC1, &joint_position_command_[1]);
    position_joint_interface_.registerHandle(jointPositionHandleC1);
    hardware_interface::JointHandle jointPositionHandleC2(jointStateHandleC2,   &joint_position_command_[2]);
    hardware_interface::JointHandle jointPositionHandleC3(jointStateHandleC3,   &joint_position_command_[3]);
    hardware_interface::JointHandle jointPositionHandleC4(jointStateHandleC4,   &joint_position_command_[4]);
    hardware_interface::JointHandle jointPositionHandleC5(jointStateHandleC5,   &joint_position_command_[5]);
    hardware_interface::JointHandle jointPositionHandleC6(jointStateHandleC6,   &joint_position_command_[6]);
    hardware_interface::JointHandle jointPositionHandleC7(jointStateHandleC7,   &joint_position_command_[7]);
    hardware_interface::JointHandle jointPositionHandleC8(jointStateHandleC8,   &joint_position_command_[8]);
    hardware_interface::JointHandle jointPositionHandleC9(jointStateHandleC9,   &joint_position_command_[9]);
    hardware_interface::JointHandle jointPositionHandleC10(jointStateHandleC10,  &joint_position_command_[10]);
    hardware_interface::JointHandle jointPositionHandleC11(jointStateHandleC11,  &joint_position_command_[11]);
    hardware_interface::JointHandle jointPositionHandleC12(jointStateHandleC12,  &joint_position_command_[12]);
    hardware_interface::JointHandle jointPositionHandleC13(jointStateHandleC13,  &joint_position_command_[13]);
    hardware_interface::JointHandle jointPositionHandleC14(jointStateHandleC14,  &joint_position_command_[14]);
    position_joint_interface_.registerHandle(jointPositionHandleC2);
    position_joint_interface_.registerHandle(jointPositionHandleC3);
    position_joint_interface_.registerHandle(jointPositionHandleC4);
    position_joint_interface_.registerHandle(jointPositionHandleC5);
    position_joint_interface_.registerHandle(jointPositionHandleC6);
    position_joint_interface_.registerHandle(jointPositionHandleC7);
    position_joint_interface_.registerHandle(jointPositionHandleC8);
    position_joint_interface_.registerHandle(jointPositionHandleC9);
    position_joint_interface_.registerHandle(jointPositionHandleC10);
    position_joint_interface_.registerHandle(jointPositionHandleC11);
    position_joint_interface_.registerHandle(jointPositionHandleC12);
    position_joint_interface_.registerHandle(jointPositionHandleC13);
    position_joint_interface_.registerHandle(jointPositionHandleC14);

    // Create Joint Limit interface for JointC
    //joint_limits_interface::getJointLimits("JointC", nh_, limits);
    //joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleC(jointPositionHandleC, limits);
    //positionJointSaturationInterface.registerHandle(jointLimitsHandleC);    

    //Register all joint interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&positionJointSaturationInterface);  

    //Create publishers
    //joint_pub_ = nh_.advertise<std_msgs::Float64>("/mujoco_ros/shoulder_pan_joint/shoulder_pan_joint_control",10);
    for (int i=0; i<pub_names_.size();i++)
      joint_pub_[i] =  nh_.advertise<std_msgs::Float64>(pub_names_[i],10);
    //Create subscriber
    //state_sub_ = nh_.subscribe("/mujoco_ros/shoulder_lift_joint/state",1, &MyControllerInterface::state_cb,this);
    state_sub_[0] = nh_.subscribe(  sub_names_[0],1,  &MyControllerInterface::state_cb_0,this);
    state_sub_[1] = nh_.subscribe(  sub_names_[1],1,  &MyControllerInterface::state_cb_1,this);
    state_sub_[2] = nh_.subscribe(  sub_names_[2],1,  &MyControllerInterface::state_cb_2,this);
    state_sub_[3] = nh_.subscribe(  sub_names_[3],1,  &MyControllerInterface::state_cb_3,this);
    state_sub_[4] = nh_.subscribe(  sub_names_[4],1,  &MyControllerInterface::state_cb_4,this);
    state_sub_[5] = nh_.subscribe(  sub_names_[5],1,  &MyControllerInterface::state_cb_5,this);
    state_sub_[6] = nh_.subscribe(  sub_names_[6],1,  &MyControllerInterface::state_cb_6,this);
    state_sub_[7] = nh_.subscribe(  sub_names_[7],1,  &MyControllerInterface::state_cb_7,this);
    state_sub_[8] = nh_.subscribe(  sub_names_[8],1,  &MyControllerInterface::state_cb_8,this);
    state_sub_[9] = nh_.subscribe(  sub_names_[9],1,  &MyControllerInterface::state_cb_9,this);
    state_sub_[10] = nh_.subscribe( sub_names_[10],1, &MyControllerInterface::state_cb_10,this);
    state_sub_[11] = nh_.subscribe( sub_names_[11],1, &MyControllerInterface::state_cb_11,this);
    state_sub_[12] = nh_.subscribe( sub_names_[12],1, &MyControllerInterface::state_cb_12,this);
    state_sub_[13] = nh_.subscribe( sub_names_[13],1, &MyControllerInterface::state_cb_13,this);
    state_sub_[14] = nh_.subscribe( sub_names_[14],1, &MyControllerInterface::state_cb_14,this);

}
void MyControllerInterface::state_cb_0(const std_msgs::Float64& msg)
{
  joint_position_temp[0]=msg.data;
}
void MyControllerInterface::state_cb_1(const std_msgs::Float64& msg)
{
  joint_position_temp[1]=msg.data;
}
void MyControllerInterface::state_cb_2(const std_msgs::Float64& msg)
{
  joint_position_temp[2]=msg.data;
}
void MyControllerInterface::state_cb_3(const std_msgs::Float64& msg)
{
  joint_position_temp[3]=msg.data;
}
void MyControllerInterface::state_cb_4(const std_msgs::Float64& msg)
{
  joint_position_temp[4]=msg.data;
}
void MyControllerInterface::state_cb_5(const std_msgs::Float64& msg)
{
  joint_position_temp[5]=msg.data;
}
void MyControllerInterface::state_cb_6(const std_msgs::Float64& msg)
{
  joint_position_temp[6]=msg.data;
}
void MyControllerInterface::state_cb_7(const std_msgs::Float64& msg)
{
  joint_position_temp[7]=msg.data;
}
void MyControllerInterface::state_cb_8(const std_msgs::Float64& msg)
{
  joint_position_temp[8]=msg.data;
}
void MyControllerInterface::state_cb_9(const std_msgs::Float64& msg)
{
  joint_position_temp[9]=msg.data;
}
void MyControllerInterface::state_cb_10(const std_msgs::Float64& msg)
{
  joint_position_temp[10]=msg.data;
}
void MyControllerInterface::state_cb_11(const std_msgs::Float64& msg)
{
  joint_position_temp[11]=msg.data;
}
void MyControllerInterface::state_cb_12(const std_msgs::Float64& msg)
{
  joint_position_temp[12]=msg.data;
}
void MyControllerInterface::state_cb_13(const std_msgs::Float64& msg)
{
  joint_position_temp[13]=msg.data;
}
void MyControllerInterface::state_cb_14(const std_msgs::Float64& msg)
{
  joint_position_temp[14]=msg.data;
}
//Control loop
void MyControllerInterface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void MyControllerInterface::read() {
  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to get the current joint position and/or velocity and/or effort       

  //from robot.
  // and fill JointStateHandle variables joint_position_[i], joint_velocity_[i] and joint_effort_[i]
  for(int i = 0; i<15; i++)
    joint_position_[i] = joint_position_temp[i];
}

void MyControllerInterface::write(ros::Duration elapsed_time) {
  // Safety
  positionJointSaturationInterface.enforceLimits(elapsed_time); // enforce limits for JointC


  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to send the commands to the robot's actuators.
  // the output commands need to send are joint_effort_command_[0] for JointA, joint_effort_command_[1] for JointB and 

  //joint_position_command_ for JointC.
  std_msgs::Float64 cmd;
  for(int i = 0; i<15; i++)
  {
    cmd.data = joint_position_command_[i];
    joint_pub_[i].publish(cmd);
  }

}