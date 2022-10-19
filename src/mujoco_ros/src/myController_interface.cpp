#include "MyController_interface.h"

MyControllerInterface::MyControllerInterface(ros::NodeHandle& nh):nh_(nh){

    // Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot.
    init();
    
    // Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
        
    //Set the frequency of the control loop.
    loop_hz_=10;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        
    //Run the control loop
    my_control_loop_ = nh_.createTimer(update_freq, &MyControllerInterface::update, this);
}

MyControllerInterface::~MyControllerInterface()
{
    
}

void MyControllerInterface::init(){

    // Create joint_state_interface for Joint
    hardware_interface::JointStateHandle jointStateHandleC("shoulder_pan_joint", &joint_position_, &joint_velocity_, &joint_effort_);
    joint_state_interface_.registerHandle(jointStateHandleC);
    // Create position joint interface as JointC accepts position command.
    hardware_interface::JointHandle jointPositionHandleC(jointStateHandleC, &joint_position_command_);
    position_joint_interface_.registerHandle(jointPositionHandleC);
    // Create Joint Limit interface for JointC
    //joint_limits_interface::getJointLimits("JointC", nh_, limits);
    //joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleC(jointPositionHandleC, limits);
    //positionJointSaturationInterface.registerHandle(jointLimitsHandleC);    

    //Register all joint interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&positionJointSaturationInterface);  

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

}

void MyControllerInterface::write(ros::Duration elapsed_time) {
  // Safety
  positionJointSaturationInterface.enforceLimits(elapsed_time); // enforce limits for JointC


  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to send the commands to the robot's actuators.
  // the output commands need to send are joint_effort_command_[0] for JointA, joint_effort_command_[1] for JointB and 

  //joint_position_command_ for JointC.

}