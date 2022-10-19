#ifndef _MUJOCO_ROS__MUJOCO_UTIL_H
#define _MUJOCO_ROS__MUJOCO_UTIL_H

#include "geometry_msgs/Pose.h"
#include "mujoco/mujoco.h"
#include "ros/ros.h"

namespace mujoco_util {

void init_mj(const ros::NodeHandle* nh);
mjtNum mj_name2id_ordie(const mjModel* m, int type, const std::string& name);
mjtNum mj_name2id_ordie(const mjModel* m, int type, const char* name);
void mj2ros_body(
    const mjModel* m, mjData* d, const char* name, geometry_msgs::Pose& ros_pose, int type);
void ros2mj_body(
    const mjModel* m, mjData* d, const char* name, const geometry_msgs::Pose& ros_pose, int type);
bool is_paused();

} // mushr_mujoco_util

#endif // _MUSHR_MUJOCO_ROS__MUSHR_MUJOCO_UTIL_H