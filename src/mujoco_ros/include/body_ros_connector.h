#ifndef _MUJOCO_ROS__BODY_ROS_CONNECTOR_H_
#define _MUJOCO_ROS__BODY_ROS_CONNECTOR_H_

#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mujoco_ros/BodyState.h>
#include <std_msgs/Float64.h> 


namespace mujoco_ros {

class BodyROSConnector
{
  public:
    BodyROSConnector(ros::NodeHandle* nh, const YAML::Node& e);

    void send_state();
    void set_body_state(mujoco_ros::BodyState& bs);

  protected:
    ros::NodeHandle* nh_;
    ros::Publisher pose_pub_;
    ros::Publisher state_pub_;
    ros::Subscriber initpose_sub_;
    std::string body_name_;
    int body_id_;
    int type_;
    std::string parent_body_name_;

    void initpose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose);
    std::string pvt_name(
        std::string name); // couldn't get private node handles working, so kludge.
};

} // namespace mushr_mujoco_ros

#endif // _MUSHR_MUJOCO_ROS__BODY_ROS_CONNECTOR_H_