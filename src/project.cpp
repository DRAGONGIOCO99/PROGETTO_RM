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
#define n_rigXD 28000
#define n_rigR 12003
#define n_col 3

std::vector<double> q_std(7,1); //variabili globali per copiare i valori del ros msg

void q_callback(const sensor_msgs::JointState &q_msg){
    q_std = q_msg.position;
}

Vector3d error_p (Vector3d &x, Vector3d &xd)
{
    Vector3d error = Vector3d::Zero();
    error = xd - x;
    return error;
}

Vector7d dq0_limits(Vector7d &q, Vector7d &q_max, Vector7d &q_min)
{
    Vector7d w = Vector7d::Zero();

    for (unsigned i=0; i<7; i++){
        w[i] = 2*q(i) / ((q_max(i) - q_min(i))*(q_max(i) - q_min(i)));
    }

    return w;

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

    Eigen::Matrix<float, n_rigXD/2, n_col> Pd1;
    Pd1.setZero();

    Eigen::Matrix<float, n_rigXD/2, n_col> Pd2;
    Pd2.setZero();

    Eigen::Matrix<float, n_rigXD/2, n_col> dot_Pd1;
    dot_Pd1.setZero();

    Eigen::Matrix<float, n_rigXD/2, n_col> dot_Pd2;
    dot_Pd2.setZero();

    Vector7d q = Vector7d::Zero();
    Matrix6d J = Matrix6d::Zero();
    Matrix4d Te = Matrix4d::Zero();
    
    Eigen::Matrix<double, 3, 7> J_p;
    J_p.setZero();
    Eigen::Matrix<double, 7, 3> J_p_inv;
    J_p_inv.setZero();
    bool stop = 1;


    Vector7d q_data = Vector7d::Zero();
    Vector3d xd = Vector3d::Zero();
    Matrix4d Ted = Matrix4d::Zero();
    Vector3d xe = Vector3d::Zero();
    Vector3d e = Vector3d::Zero();
    Vector7d q_dot = Vector7d::Zero();
    Vector3d dxd = Vector3d::Zero();
    Eigen::Matrix<double, 7, 7> Id = MatrixXd::Identity(7,7);
    Vector7d dq0 = Vector7d::Zero();

    Vector7d q_max = Vector7d::Zero();
    Vector7d q_min = Vector7d::Zero();
    Vector3d eo = Vector3d::Zero();

    double K = 10.0;
    std_msgs::Float64 q_cmd[7];

    q_data[0] = -0.575056;
    q_data[1] = 0.199522;
    q_data[2] = 0.602169;
    q_data[3] = -1.70087;
    q_data[4] = 0.107438;
    q_data[5] = 0.88287;
    q_data[6] = -0.000148029;

    for(int i=0; i<7; i++){
            q_cmd[i].data = q_data[i];
            joint_pos_cmd[i].publish(q_cmd[i]);
     }
    cout <<"Home configuration setted\n"<<endl;

    fstream file1("/home/dev/ros1_ws/src/PROGETTO_RM/src/Xd.txt",ios::in);
    if(!file1){
    cout<<"errore"<<endl;
        }
    
    else{
        for(int i=0;i<n_rigXD/2;i++){
            for(int j=0;j<n_col;j++){
                file1>>Pd1(i,j);
                cout<<"sto salvando i dati"<<endl;
            }
        }

        for(int i=0;i<n_rigXD/2;i++){
            for(int j=0;j<n_col;j++){
                file1>>Pd2(i,j);
                cout<<"sto salvando i dati"<<endl;
            }
        }
       
    }

 ifstream file2("/home/dev/ros1_ws/src/PROGETTO_RM/src/Xd_dot.txt",ios::in);
    if(!file2){
    cout<<"errore"<<endl;
    }
    else{
        for(int i=0;i<n_rigXD/2;i++){
          for(int j=0;j<n_col;j++){
             file2>>dot_Pd1(i,j);
             cout<<"sto salvando i dati"<<endl;
          }
       }

        for(int i=0;i<n_rigXD/2;i++){
          for(int j=0;j<n_col;j++){
             file2>>dot_Pd2(i,j);
             cout<<"sto salvando i dati"<<endl;
          }
       }
    }

    kuka_robot kuka; 
    int p=0;
    cout << "Press enter to start the trajectory execution" << endl;
	string ln;
	getline(cin, ln);
    while(ros::ok()) {

        if (p<n_rigXD){

        

            if(q_std.size() > 0){ //solo quando effettivamente mi arrivano valori riempi
                q << q_std[0], q_std[1], q_std[2], q_std[3], q_std[4], q_std[5], q_std[6];
            }

            if (p<n_rigXD/2) {
                xd(0)=Pd1(p,0);
                xd(1)=Pd1(p,1);
                xd(2)=Pd1(p,2);

                dxd(0)=dot_Pd1(p,0);
                dxd(1)=dot_Pd1(p,1);
                dxd(2)=dot_Pd1(p,2);



            } else {
                xd(0)=Pd2(p-n_rigXD/2,0);
                xd(1)=Pd2(p-n_rigXD/2,1);
                xd(2)=Pd2(p-n_rigXD/2,2);

                dxd(0)=dot_Pd2(p-n_rigXD/2,0);
                dxd(1)=dot_Pd2(p-n_rigXD/2,1);
                dxd(2)=dot_Pd2(p-n_rigXD/2,2);

            }


            Te = kuka.Te(q);
            J = kuka.jacobian(q);
            J_p = J.block(0,0,3,7);
            J_p_inv = (J_p.transpose())*(J_p*J_p.transpose()).inverse();
            
            /*cout << "Te" << endl;
            cout << Te << endl;
            cout << "Jacobian" << endl;
            cout << J << endl;*/

            /*q_data[0] = -0.46;
            q_data[1] = 0.33;
            q_data[2] = -0.51;
            q_data[3] = -1.5;
            q_data[4] = 0.16;
            q_data[5] = 1.33;
            q_data[6] = -0.99;*/

            //Ted = kuka.Te(q_data);
            //xd <<0.5,0,0.5;
            xe = Te.block(0,3,3,1);

            cout << "x" << endl;
            cout << xe << endl;

            /*cout << "TE " << endl;
            cout << Te <<endl;
            cout << " Re "<<endl;
            cout << Te.block(0,0,3,3) <<endl;*/

            cout << "xd" << endl;
            cout << xd << endl;

            e = error_p(xe, xd);

            q_max << 2.69706, 2.0944, 2.69706, 2.0944, 2.69706, 2.0944, 3.05433;
            q_min << -2.69706, -2.0944, -2.69706, -2.0944, -2.69706, -2.0944, -3.05433;

            dq0 = dq0_limits(q, q_max, q_min);

            q_dot = J_p_inv*(dxd + K*e); //+ (Id - J_p_inv*J_p)*dq0;

            q = q + q_dot*0.001;

            cout<< "q "<<endl;
            cout<< q <<endl;

            for(int i=0; i<7; i++){
                q_cmd[i].data = q[i];
                joint_pos_cmd[i].publish(q_cmd[i]);
            }

            /*if (e.norm()<0.005){
                stop = 0;
            }*/
            
            ros::spinOnce();
            rate.sleep();
            p++;
        }
    }

    return 0;
}
