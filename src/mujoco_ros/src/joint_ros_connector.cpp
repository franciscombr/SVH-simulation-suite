#include "joint_ros_connector.h"
#include "body_ros_connector.h"
#include "mjglobal.h"
#include "mujoco_util.h"
#include <iostream>

namespace mujoco_ros {

JointROSConnector::JointROSConnector(ros::NodeHandle* nh, const YAML::Node& e): BodyROSConnector{nh, e}
{
    // check if the car body exists
    mjModel* m = mjglobal::mjmodel();
    joint_angle_ = 0.0;

    std::string control_topic = "control";
    if (e["control_topic"])
    {
        control_topic = e["control_topic"].as<std::string>();
    }
    control_sub_ = nh_->subscribe(
        pvt_name(control_topic), 1, &JointROSConnector::control_cb, this);

    joint_ctrl_idx_ = mujoco_util::mj_name2id_ordie(m, mjOBJ_ACTUATOR, body_name_);

    joint_state_idx_ = mujoco_util::mj_name2id_ordie(m, mjOBJ_JOINT, body_name_);

    if (e["p_gain"])
    {
        p_gain_=e["p_gain"].as<double>();
    }
    if (e["d_gain"])
    {
        d_gain_=e["d_gain"].as<double>();
    }

    type_ = mjOBJ_JOINT;
    
}

void JointROSConnector::send_state()
{
    mjModel* m = mjglobal::mjmodel();
    mjData* d = mjglobal::mjdata_lock();

    std_msgs::Float64 joint_state_msg;

    joint_state_msg.data=d->qpos[joint_state_idx_];
    //joint_state_msg.position[0] = d->qpos[joint_state_idx_];

    
    state_pub_.publish(joint_state_msg);

    mjglobal::mjdata_unlock();
}

void JointROSConnector::set_body_state(mujoco_ros::BodyState& bs)
{
    BodyROSConnector::set_body_state(bs);

    bs.ctrl_joint_angle = joint_angle_;
    /*
    mjData* d = mjglobal::mjdata_lock();
    get_gyro(d, bs.imu);
    get_velocimeter(d, bs.velocity);
    mjglobal::mjdata_unlock();
    */
}

//Esta função manda os valores de referência para o controlador do atuador em mujoco
void JointROSConnector::mujoco_controller()
{
    mjData* d = mjglobal::mjdata_lock();
    

    d->ctrl[joint_ctrl_idx_] = -p_gain_*(d->qpos[joint_state_idx_]-joint_angle_)-d_gain_*d->qvel[joint_state_idx_];
    //std::cout << joint_ctrl_idx_ << ": "<< joint_angle_ << "real:" << d->qpos[joint_state_idx_] << "\n";

    mjglobal::mjdata_unlock();
}

//receber ordens vindas do ros master
void JointROSConnector::control_cb(
    const std_msgs::Float64& msg)
{
    joint_angle_ = msg.data;

}

//inutil
std::string JointROSConnector::joint_ref(const std::string name)
{
    return body_name_+"_"+ name;
}

} 