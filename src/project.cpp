#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include "std_msgs/Float64.h"

#include <tf/transform_broadcaster.h>

#include <math.h>
#include <cmath>
#include <sstream>
#include <stdio.h>

#include "/home/dev/ros1_ws/src/PROGETTO_RM/include/kuka_kine.h"

using namespace Eigen;


std::vector<double> q_std(7,1); //variabili globali per copiare i valori del ros msg

void q_callback(const sensor_msgs::JointState &q_msg){
    q_std = q_msg.position;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "kuka");

    ros::NodeHandle nh;
    ros::Rate rate(100);

    ros::Subscriber q_sub = nh.subscribe("/iiwa/joint_states",1000, q_callback);
    ros::Publisher joint_pos_cmd[7];

    joint_pos_cmd[0] = nh.advertise<std_msgs::Float64>("/iiwa/joint1_position_controller/command", 1000);
    joint_pos_cmd[1] = nh.advertise<std_msgs::Float64>("/iiwa/joint2_position_controller/command", 1000);
    joint_pos_cmd[2] = nh.advertise<std_msgs::Float64>("/iiwa/joint3_position_controller/command", 1000);
    joint_pos_cmd[3] = nh.advertise<std_msgs::Float64>("/iiwa/joint4_position_controller/command", 1000);
    joint_pos_cmd[4] = nh.advertise<std_msgs::Float64>("/iiwa/joint5_position_controller/command", 1000);
    joint_pos_cmd[5] = nh.advertise<std_msgs::Float64>("/iiwa/joint6_position_controller/command", 1000);
    joint_pos_cmd[6] = nh.advertise<std_msgs::Float64>("/iiwa/joint7_position_controller/command", 1000);

    Vector7d q = Vector7d::Zero();
    Matrix6d J = Matrix6d::Zero();
    Matrix4d Te = Matrix4d::Zero();
    Vector3d x = Vector3d::Zero();

    kuka_robot kuka; 

    while(ros::ok()) {

        if(q_std.size() > 0){ //solo quando effettivamente mi arrivano valori riempi
            q << q_std[0], q_std[1], q_std[2], q_std[3], q_std[4], q_std[5], q_std[6];
        }

        Te = kuka.Te(q);

        x = Te.block(0,3,3,1);

        cout << "x" << endl;
        cout << x << endl;

        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}
