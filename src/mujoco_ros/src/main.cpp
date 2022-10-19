#include "mujoco/mujoco.h"
#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

#include "mjglobal.h"
#include "mujoco_util.h"
#include "body_ros_connector.h"
#include "joint_ros_connector.h"
#include "mujoco_ros/BodyStateArray.h"
#include "simple_viz.h"
#include "MyController_interface.h"
#include <controller_manager/controller_manager.h>

#include <fstream>

bool mj_sim_pause = false;


int main(int argc, char** argv)
{
    

    std::string model_file_path;
    bool do_viz;

    ros::init(argc, argv, "mujoco_ros");
    ros::NodeHandle nh("~");
    
    

    if (!nh.getParam("model_file_path", model_file_path))
    {
        ROS_FATAL("%s not set", nh.resolveName("model_file_path").c_str());
        exit(1);
    }
    if (!nh.getParam("viz", do_viz))
    {
        ROS_FATAL("%s not set", nh.resolveName("viz").c_str());
        exit(1);
    }

    ROS_INFO("Loading model");
    char* error;
    if (mjglobal::init_model(model_file_path.c_str(), &error))
    {
        ROS_FATAL("Could not load binary model %s", error);
        exit(1);
    }
    ROS_INFO("Loading data");
    mjglobal::init_data();
    ROS_INFO("Loaded model and data");


     std::string config_file;
    if (!nh.getParam("config_file_path", config_file))
    {
        ROS_FATAL("%s not set", nh.resolveName("config_file_path").c_str());
        exit(1);
    }

    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_file);
    }
    catch (YAML::BadFile e)
    {
        ROS_INFO("Couldn't open file %s", config_file.c_str());
        exit(1);
    }
    catch (std::exception e)
    {
        ROS_INFO("Unknown exception opening config file");
        exit(1);
    }

    std::vector<mujoco_ros::JointROSConnector*> joint_conn;
    std::vector<mujoco_ros::BodyROSConnector*> body_conn;

    ROS_INFO("Loading robot configuration");

    if(config["joints"])
    {
        YAML::Node joint_cfg = config["joints"];
        for(int i=0; i<joint_cfg.size(); i++)
        {
            joint_conn.push_back(
                new mujoco_ros::JointROSConnector(&nh, joint_cfg[i]));
        }
    }

    ROS_INFO("Loading bodies configuration");
    // Load body info
    if (config["bodies"])
    {
        YAML::Node bodies_cfg = config["bodies"];
        for (int i = 0; i < bodies_cfg.size(); i++)
        {
            body_conn.push_back(
                new mujoco_ros::BodyROSConnector(&nh, bodies_cfg[i]));
        }
    }

    ros::Publisher body_state_pub
        = nh.advertise<mujoco_ros::BodyStateArray>("body_state", 10);



    
    if (do_viz)
    {
        ROS_INFO("Starting visualization");
        viz::init();
    }

    mjModel* m = mjglobal::mjmodel();
    mjData* d = NULL;

    double t = 0.0;
    const double dt = m->opt.timestep;

    float maxrate = 60.0;
    ros::Rate simrate(maxrate);
    while (ros::ok())
    {
        m = mjglobal::mjmodel();
        d = mjglobal::mjdata_lock();

        mjtNum simstart = d->time;
        if (!mujoco_util::is_paused())
        {
            while (d->time - simstart < 1.0 / maxrate)
            {
                mj_step1(m, d);
                for (auto cc : joint_conn)
                    cc->mujoco_controller();
                mj_step2(m, d);
            }
        }

        mujoco_ros::BodyStateArray body_state;
        body_state.simtime = d->time;
        body_state.header.stamp = ros::Time::now();

        int i = 0;
        for (int j = 0; i < joint_conn.size(); j++)
        {
            auto cc = joint_conn[j];
            cc->send_state(); //passar aqui o tipo para distinguir entre articulações e corpos

            mujoco_ros::BodyState bs;
            cc->set_body_state(bs);
            body_state.states.push_back(bs);
            i++;
        }
        for (int j = 0; j < body_conn.size(); j++)
        {
            auto bc = body_conn[j];
            bc->send_state();

            mujoco_ros::BodyState bs;
            bc->set_body_state(bs);
            body_state.states.push_back(bs);
            i++;
        }

        body_state_pub.publish(body_state);
        mjtNum * sens = d->sensordata;
        std::cout << d->time << ","<<*sens<<"\n";


        mjglobal::mjdata_unlock();

        if (do_viz)
            viz::display();

        ros::spinOnce();
        simrate.sleep();
    }

    for (auto cc : joint_conn)
        delete cc;
    for (auto bc : body_conn)
        delete bc;

    // free MuJoCo model and data, deactivate
    mjglobal::delete_model_and_data();
    mj_deactivate();

    viz::destroy();

}