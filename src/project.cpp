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
#define num_traj 7
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
    ros::Rate rate(300);

    ros::Subscriber q_sub = nh.subscribe("/iiwa/joint_states",1000, q_callback);
    ros::Publisher joint_pos_cmd[7];

    joint_pos_cmd[0] = nh.advertise<std_msgs::Float64>("/iiwa/joint1_position_controller/command", 1000);
    joint_pos_cmd[1] = nh.advertise<std_msgs::Float64>("/iiwa/joint2_position_controller/command", 1000);
    joint_pos_cmd[2] = nh.advertise<std_msgs::Float64>("/iiwa/joint3_position_controller/command", 1000);
    joint_pos_cmd[3] = nh.advertise<std_msgs::Float64>("/iiwa/joint4_position_controller/command", 1000);
    joint_pos_cmd[4] = nh.advertise<std_msgs::Float64>("/iiwa/joint5_position_controller/command", 1000);
    joint_pos_cmd[5] = nh.advertise<std_msgs::Float64>("/iiwa/joint6_position_controller/command", 1000);
    joint_pos_cmd[6] = nh.advertise<std_msgs::Float64>("/iiwa/joint7_position_controller/command", 1000);


    std::vector<Vector3d> Pd(n_rigXD);
    std::vector<Vector3d> dot_Pd(n_rigXD);
    std::vector<Vector3d> wd(n_rigXD);
    std::vector<Matrix3d> Rd(n_rigXD);
    
    
    Vector7d q = Vector7d::Zero();
    Matrix6d J = Matrix6d::Zero();
    Matrix4d Te = Matrix4d::Zero();
    
    Eigen::Matrix<double, 3, 7> J_p;
    J_p.setZero();

    Eigen::Matrix<double, 7, 3> J_p_inv;
    J_p_inv.setZero();
   
    Eigen::Matrix<double, 7, 6> J_inv;
    J_inv.setZero();


    Vector7d q_data = Vector7d::Zero();
    Vector3d xd = Vector3d::Zero();
    Matrix3d R_d = Matrix3d::Zero();
    Matrix3d R_e = Matrix3d::Zero();
    Vector3d w_d = Vector3d::Zero();
    Vector3d xe = Vector3d::Zero();
    Vector3d ep = Vector3d::Zero();
    Vector3d eo = Vector3d::Zero();

    Vector6d e = Vector6d::Zero();
    Vector6d dot_xd = Vector6d::Zero();
    Vector4d Q_d = Vector4d::Zero();
    Vector4d Q_e = Vector4d::Zero();
    Vector7d q_dot = Vector7d::Zero();
    Vector3d dxd = Vector3d::Zero();
    Eigen::Matrix<double, 7, 7> Id = MatrixXd::Identity(7,7);
    Eigen::Matrix<double, 6, 6> K = MatrixXd::Identity(6,6);
    Vector7d dq0 = Vector7d::Zero();

    Vector7d q_max = Vector7d::Zero();
    Vector7d q_min = Vector7d::Zero();
    
    double Kp = 50;
    K(0,0) = 100;
    K(1,1) = 100;
    K(2,2) = 100;
    K(3,3) = 120;
    K(4,4) = 120;
    K(5,5) = 120;

    std_msgs::Float64 q_cmd[7];

    q_data[0] = -0.575056;
    q_data[1] = 0.199522;
    q_data[2] = 0.602169;
    q_data[3] = -1.70087;
    q_data[4] = 0.107438;
    q_data[5] = 0.88287;
    q_data[6] = -0.000148029;

    for(int j=0;j<10 ; j++)
    {
    for(int i=0; i<7; i++){
            q_cmd[i].data = q_data[i];
            joint_pos_cmd[i].publish(q_cmd[i]);
     }
     rate.sleep();
     
    }
    cout <<"Home configuration setted\n"<<endl;

    ifstream file1("/home/dev/ros1_ws/src/PROGETTO_RM/src/Xd.txt",ios::in);
    if(!file1){
    cout<<"errore"<<endl;
        }
    
    else{
        for(int i=0;i<n_rigXD;i++){
            for(int j=0;j<n_col;j++){
                file1>>Pd[i](j);
                cout<<"sto salvando i dati"<<endl;
            }
        }

        
       
    }


ifstream file2("/home/dev/ros1_ws/src/PROGETTO_RM/src/Xd_dot.txt",ios::in);
    if(!file2){
    cout<<"errore"<<endl;
    }
    else{
        for(int i=0;i<n_rigXD;i++){
          for(int j=0;j<n_col;j++){
             file2>>dot_Pd[i](j);
             cout<<"sto salvando i dati"<<endl;
          }
       }
    }
    
    
    ifstream file3("/home/dev/ros1_ws/src/PROGETTO_RM/src/R_01d.txt",ios::in);
    if(!file3){
    cout<<"errore"<<endl;
    }
    else{
        for(int i=0;i<n_rigR/3;i++){
          for(int j=0;j<n_col;j++){
             file3>>Rd[i](j,0);
             file3>>Rd[i](j,1);
             file3>>Rd[i](j,2);
             cout<<"sto salvando i dati"<<endl;
          }
       }
    }
    
    ifstream file4("/home/dev/ros1_ws/src/PROGETTO_RM/src/R_12d.txt",ios::in);
    if(!file4){
    cout<<"errore"<<endl;
    }
    else{
        for(int i=n_rigR/3;i<2*n_rigR/3;i++){
          for(int j=0;j<n_col;j++){
             file4>>Rd[i](j,0);
             file4>>Rd[i](j,1);
             file4>>Rd[i](j,2);
             cout<<"sto salvando i dati"<<endl;
          }
  
       }
    }
    
 ifstream file5("/home/dev/ros1_ws/src/PROGETTO_RM/src/R_23d.txt",ios::in);
    if(!file5){
    cout<<"errore"<<endl;
    }
    else{
        for(int i=2*n_rigR/3;i<3*n_rigR/3;i++){
          for(int j=0;j<n_col;j++){
             file5>>Rd[i](j,0);
             file5>>Rd[i](j,1);
             file5>>Rd[i](j,2);
             cout<<"sto salvando i dati"<<endl;
          }
  
       }
    }

for(int i=3*n_rigR/3;i<4*n_rigR/3;i++){
            Rd[i] =Rd[3*n_rigR/3-1];
          }
    
ifstream file6("/home/dev/ros1_ws/src/PROGETTO_RM/src/R_45d.txt",ios::in);
    if(!file6){
    cout<<"errore"<<endl;
    }
    else{
        for(int i=4*n_rigR/3;i<5*n_rigR/3;i++){
          for(int j=0;j<n_col;j++){
             file6>>Rd[i](j,0);
             file6>>Rd[i](j,1);
             file6>>Rd[i](j,2);
             cout<<"sto salvando i dati"<<endl;
          }
  
       }
    }
  
 
for(int i=5*n_rigR/3;i<7*n_rigR/3;i++){
            Rd[i] =Rd[5*n_rigR/3-1];
          }

ifstream file7("/home/dev/ros1_ws/src/PROGETTO_RM/src/w1.txt",ios::in);
    if(!file7){
    cout<<"errore"<<endl;
    }
    else{
        for(int i=0;i<n_rigXD/num_traj;i++){
          for(int j=0;j<n_col;j++){
             file7>>wd[i](j);
             cout<<"sto salvando i dati"<<endl;
          }
       }
    }
    ifstream file8("/home/dev/ros1_ws/src/PROGETTO_RM/src/w2.txt",ios::in);
    if(!file8){
    cout<<"errore"<<endl;
    }
    else{
        for(int i=n_rigXD/num_traj;i<2*n_rigXD/num_traj;i++){
          for(int j=0;j<n_col;j++){
             file8>>wd[i](j);
             cout<<"sto salvando i dati"<<endl;
          }
       }
    }
    
    ifstream file9("/home/dev/ros1_ws/src/PROGETTO_RM/src/w3.txt",ios::in);
    if(!file9){
    cout<<"errore"<<endl;
    }
    else{
        for(int i=2*n_rigXD/num_traj;i<3*n_rigXD/num_traj;i++){
          for(int j=0;j<n_col;j++){
             file9>>wd[i](j);
             cout<<"sto salvando i dati"<<endl;
          }
       }
    }
    for(int i=3*n_rigXD/num_traj;i<4*n_rigXD/num_traj;i++){
            wd[i]=wd[3*n_rigXD/num_traj-1] ;
            cout<<"sto salvando i dati"<<endl;
          }
    ifstream file10("/home/dev/ros1_ws/src/PROGETTO_RM/src/w4.txt",ios::in);
    if(!file10){
    cout<<"errore"<<endl;
    }
    else{
        for(int i=4*n_rigXD/num_traj;i<5*n_rigXD/num_traj;i++){
          for(int j=0;j<n_col;j++){
             file10>>wd[i](j);
             cout<<"sto salvando i dati"<<endl;
          }
       }
    }
    for(int i=5*n_rigXD/num_traj;i<7*n_rigXD/num_traj;i++){
            wd[i]=wd[5*n_rigXD/num_traj-1] ;
            cout<<"sto salvando i dati"<<endl;
            cout << wd[i]<<endl;
          
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

            
            xd(0)=Pd[p](0);
            xd(1)=Pd[p](1);
            xd(2)=Pd[p](2);

            dxd(0)=dot_Pd[p](0);
            dxd(1)=dot_Pd[p](1);
            dxd(2)=dot_Pd[p](2);

            R_d = Rd[p];
            cout << "R_d" <<endl;
            cout << R_d <<endl;
            w_d(0) = wd[p](0);
            w_d(1) = wd[p](1);
            w_d(2) = wd[p](2);
            
            Te = kuka.Te(q);
            J = kuka.jacobian(q);
            J_p = J.block(0,0,3,7);
            //J_p_inv = (J_p.transpose())*(J_p*J_p.transpose()).inverse();
            J_inv = (J.transpose())*(J*J.transpose()).inverse();
            
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
            R_e = Te.block(0,0,3,3);

            cout << "R_e" <<endl;
            cout << R_e <<endl;

            // Quaternion extraction
            Q_d = kuka.Rot2Quat(R_d);
            Q_e = kuka.Rot2Quat(R_e);

            eo=kuka.QuatError(Q_d, Q_e);

            cout << "x" << endl;
            cout << xe << endl;

            /*cout << "TE " << endl;
            cout << Te <<endl;
            cout << " Re "<<endl;
            cout << Te.block(0,0,3,3) <<endl;*/

            cout << "xd" << endl;
            cout << xd << endl;

            ep = error_p(xe, xd);

           cout << "ep" << endl;
            cout << ep << endl;

            cout << "\neo"<<endl;
            cout << eo << endl; 

            q_max << 2.69706, 2.0944, 2.69706, 2.0944, 2.69706, 2.0944, 3.05433;
            q_min << -2.69706, -2.0944, -2.69706, -2.0944, -2.69706, -2.0944, -3.05433;

            dq0 = dq0_limits(q, q_max, q_min);

            e(0) = ep(0);
            e(1) = ep(1);
            e(2) = ep(2);
            e(3) = eo(0);
            e(4) = eo(1);
            e(5) = eo(2);

            cout<< "e "<< endl;
            cout << e<<endl;

            dot_xd(0) = dxd(0);
            dot_xd(1) = dxd(1);
            dot_xd(2) = dxd(2);
            dot_xd(3) = w_d(0);
            dot_xd(4) = w_d(1);
            dot_xd(5) = w_d(2);

            //q_dot = J_p_inv*(dxd + Kp*ep); //+ (Id - J_p_inv*J_p)*dq0;
            q_dot = J_inv*(dot_xd + K*e);


            q = q + q_dot*0.001;

            cout<< "q "<<endl;
            cout<< q <<endl;

            //for(int j=0;j<10 ; j++)
            //{
            for(int i=0; i<7; i++){
                q_cmd[i].data = q[i];
                joint_pos_cmd[i].publish(q_cmd[i]);
            }
            //}

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
