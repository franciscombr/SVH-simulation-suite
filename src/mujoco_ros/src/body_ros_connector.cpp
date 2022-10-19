#include "body_ros_connector.h"
#include "mjglobal.h"
#include "mujoco_util.h"

namespace mujoco_ros {

BodyROSConnector::BodyROSConnector(ros::NodeHandle* nh, const YAML::Node& e)
{
    if (!e["name"])
    {
        ROS_FATAL("No 'name' for some element");
        exit(1);
    }
    body_name_ = e["name"].as<std::string>();

    nh_ = nh;

    std::string pose_topic = "pose";
    if (e["pose_topic"])
    {
        pose_topic = e["pose_topic"].as<std::string>();
    }
    std::string state_topic = "state";
    if (e["state_topic"])
    {
        state_topic = e["state_topic"].as<std::string>();
    }
    ROS_INFO("pose topic resolve %s", nh_->resolveName(pvt_name(pose_topic)).c_str());
    pose_pub_ = nh_->advertise<geometry_msgs::PoseStamped>(pvt_name(pose_topic), 10);
    state_pub_ = nh_->advertise<std_msgs::Float64>(pvt_name(state_topic),10);

    std::string initpose_topic = "initialpose";
    if (e["initialpose_topic"])
    {
        initpose_topic = e["initialpose_topic"].as<std::string>();
    }
    initpose_sub_ = nh_->subscribe(
        pvt_name(initpose_topic), 1, &BodyROSConnector::initpose_cb, this);

    mjModel* m = mjglobal::mjmodel();
    body_id_ = mujoco_util::mj_name2id_ordie(m, mjOBJ_JOINT, body_name_);

    int parent_body_id = m->body_parentid[body_id_];
    if (parent_body_id == 0)
    {
        parent_body_name_ = "map";
    }
    else
    {
        std::string parent_body_name_(mj_id2name(m, mjOBJ_BODY, parent_body_id));
    }

    type_ = mjOBJ_BODY;
}

void BodyROSConnector::send_state()
{
    mjModel* m = mjglobal::mjmodel();
    mjData* d = mjglobal::mjdata_lock();

    geometry_msgs::PoseStamped ros_pose;

    ros_pose.header.frame_id = parent_body_name_;
    mujoco_util::mj2ros_body(m, d, body_name_.c_str(), ros_pose.pose, type_);

    pose_pub_.publish(ros_pose);

    mjglobal::mjdata_unlock();
}

void BodyROSConnector::set_body_state(mujoco_ros::BodyState& bs)
{
    bs.name = body_name_;

    mjModel* m = mjglobal::mjmodel();
    mjData* d = mjglobal::mjdata_lock();

    mujoco_util::mj2ros_body(m, d, body_name_.c_str(), bs.pose, type_);

    mjglobal::mjdata_unlock();
}

void BodyROSConnector::initpose_cb(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    mjModel* m = mjglobal::mjmodel();
    mjData* d = mjglobal::mjdata_lock();

    ROS_INFO(
        "[%s]   init pose: Position(%.3f, %.3f, %.3f), Orientation(%.3f, "
        "%.3f, %.3f, %.3f)",
        body_name_.c_str(),
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    mujoco_util::ros2mj_body(m, d, body_name_.c_str(), msg->pose.pose, type_);

    mjglobal::mjdata_unlock();
}

std::string BodyROSConnector::pvt_name(std::string name)
{
    return body_name_ + "/" + name;
}

} 