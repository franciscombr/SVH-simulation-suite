#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>

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

    double joint_position_;
    double joint_velocity_;
    double joint_effort_;
    double joint_position_command_;
    ros::NodeHandle nh_;
    ros::Timer my_control_loop_;
    ros::Duration elapsed_time_;
    double loop_hz_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

};