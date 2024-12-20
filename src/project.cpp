#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include "std_msgs/Float64.h"
#include <string> // Per std::string

#include <tf/transform_broadcaster.h>

#include <math.h>
#include <cmath>
#include <sstream>
#include <stdio.h>

#include "/home/dev/ros1_GB/src/PROGETTO_RM/include/kuka_kine.h"
std::string folder = "/home/dev/ros1_GB/src/PROGETTO_RM/src/" ;

using namespace Eigen;

#define n_rigXD 3200
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




int main(int argc, char** argv)
{
    ros::init(argc, argv, "kuka");

    ros::NodeHandle nh;
    ros::Rate rate(50);

    ros::Subscriber q_sub = nh.subscribe("/iiwa/joint_states",1, q_callback);
    ros::Publisher joint_pos_cmd[7];

    joint_pos_cmd[0] = nh.advertise<std_msgs::Float64>("/iiwa/joint1_position_controller/command", 1); 
    joint_pos_cmd[1] = nh.advertise<std_msgs::Float64>("/iiwa/joint2_position_controller/command", 1);
    joint_pos_cmd[2] = nh.advertise<std_msgs::Float64>("/iiwa/joint3_position_controller/command", 1);
    joint_pos_cmd[3] = nh.advertise<std_msgs::Float64>("/iiwa/joint4_position_controller/command", 1);
    joint_pos_cmd[4] = nh.advertise<std_msgs::Float64>("/iiwa/joint5_position_controller/command", 1);
    joint_pos_cmd[5] = nh.advertise<std_msgs::Float64>("/iiwa/joint6_position_controller/command", 1);
    joint_pos_cmd[6] = nh.advertise<std_msgs::Float64>("/iiwa/joint7_position_controller/command", 1);


    std::vector<Vector3d> Pd(n_rigXD);
    std::vector<Vector3d> dot_Pd(n_rigXD);
 
    
    
    Vector7d q = Vector7d::Zero();
    Matrix6d J = Matrix6d::Zero();
    Matrix6d J_inc = Matrix6d::Zero();
    Matrix6d Ja = Matrix6d::Zero();
    Matrix4d Te = Matrix4d::Zero();
    
    Eigen::Matrix<double, 3, 7> J_p;
    J_p.setZero();

    Eigen::Matrix<double, 3, 7> J_o;
    J_o.setZero();

    Eigen::Matrix<double, 7, 3> J_p_inv;
    J_p_inv.setZero();
   
    Eigen::Matrix<double, 7, 6> J_inv;
    J_inv.setZero();

    //Transformation matrix for euler angles
    Matrix3d T = Matrix3d::Zero();
    Vector7d q_data = Vector7d::Zero();
    Vector6d xd = Vector6d::Zero();
    Vector3d xdp = Vector3d::Zero();
    

    Matrix3d R_e = Matrix3d::Zero();

    Vector6d xe = Vector6d::Zero();
    Vector3d xep = Vector3d::Zero();
    Vector3d ep = Vector3d::Zero();
   

    Vector6d z = Vector6d::Zero();
    Vector6d dz = Vector6d::Zero();
    Vector6d dz_old = Vector6d::Zero();
    Vector6d ddz = Vector6d::Zero();


    Vector3d Euler_d = Vector3d::Zero();
    Vector3d Euler_e = Vector3d::Zero();
    Vector6d e = Vector6d::Zero();
   

    Vector7d q_dot = Vector7d::Zero();
    Vector6d dxd = Vector6d::Zero();

    //Velocità terna cedevole
    Vector6d dxt = Vector6d::Zero();

    Eigen::Matrix<double, 7, 7> Id = MatrixXd::Identity(7,7);
    Eigen::Matrix<double, 6, 6> K;
    K.setZero();

    Vector7d dq0 = Vector7d::Zero();
    Vector7d grad_w = Vector7d::Zero();

    Vector7d q_inc = Vector7d::Zero();
    
    std::vector<double> he_sin(n_rigXD);
    Vector6d he = Vector6d::Zero();
    

    // Forza costante
    he<<0,5,0,0,0,0;

    // Torque costante
    //he<<0,0,0,0,0,2;


    // Parametri controllo ammettenza

    Eigen::Matrix<double, 6, 6> Md = MatrixXd::Identity(6,6);
    
   
    
    Matrix<double,6,6> invMd;
    invMd = Md.inverse();
    
    
    Eigen::Matrix<double, 6, 6> Kd = MatrixXd::Identity(6,6);
    Kd(0,0)=10;
    Kd(1,1)=100;
    Kd(2,2)=10;
    Kd(3,3)=10;
    Kd(4,4)=10;
    Kd(5,5)=10;

    Eigen::Matrix<double, 6, 6> Kpa = MatrixXd::Identity(6,6);
    Kpa(0,0)=10;
    Kpa(1,1)=500;
    Kpa(2,2)=10;
    Kpa(3,3)=10;
    Kpa(4,4)=10;
    Kpa(5,5)=10;

    float w_inc = 0.0;
    float w_now =0.0;
    
 
    
    double k0 = 10;

    //Gain click
    K(0,0) = 100;
    K(1,1) = 100;
    K(2,2) = 100;
    K(3,3) = 100;
    K(4,4) = 100;
    K(5,5) = 100;

    std_msgs::Float64 q_cmd[7];

    q_data[0] = -0.575056;
    q_data[1] = 0.199522;
    q_data[2] = 0.602169;
    q_data[3] = -1.70087;
    q_data[4] = 0.107438;
    q_data[5] = 0.88287;
    q_data[6] = -0.000148029;

    for (int j=0; j<20; j++){
    for(int i=0; i<7; i++){
            q_cmd[i].data = q_data[i];
            joint_pos_cmd[i].publish(q_cmd[i]);
     }
     rate.sleep();
    }

    cout <<"Home configuration setted\n"<<endl;

    ifstream file1(folder+"Xd.txt",ios::in);
    if(!file1){
    cout<<"errore"<<endl;
        }
    
    else{
        for(int i=0;i<n_rigXD;i++){
            for(int j=0;j<n_col;j++){
                file1>>Pd[i](j);
                //cout<<"sto salvando i dati"<<endl;
            }
        }

        
       
    }


ifstream file2(folder+"Xd_dot.txt",ios::in);
    if(!file2){
    cout<<"errore"<<endl;
    }
    else{
        for(int i=0;i<n_rigXD;i++){
          for(int j=0;j<n_col;j++){
             file2>>dot_Pd[i](j);
             //cout<<"sto salvando i dati"<<endl;
          }
       }
    }

    ifstream file3(folder+"he_sin.txt",ios::in);
    if(!file3){
    cout<<"errore"<<endl;
    }
    else{
        for(int i=0;i<n_rigXD;i++){
             file3>>he_sin[i];

       }
    }
    


    kuka_robot kuka; 



    int p=0;

    
    cout << "Press enter to start the trajectory execution" << endl;
	string ln;
	getline(cin, ln);


    while(ros::ok()) {

        if (p<n_rigXD){

            ros::spinOnce();

            if(q_std.size() > 0){ //solo quando effettivamente mi arrivano valori riempi
                q << q_std[0], q_std[1], q_std[2], q_std[3], q_std[4], q_std[5], q_std[6];
            }

            
            cout<< "q" <<endl;
            cout<< q <<endl;

            
            xd(0) = Pd[p](0);
            xd(1) = Pd[p](1);
            xd(2) = Pd[p](2);

            dxd(0) = dot_Pd[p](0);
            dxd(1) = dot_Pd[p](1);
            dxd(2) = dot_Pd[p](2);
            dxd(3) = 0;
            dxd(4) = 0;
            dxd(5) = 0;

            //Forza sinusoidale
            //he(0)=he_sin[p];


            
            Te = kuka.Te(q);
            R_e = Te.block(0,0,3,3);
            J = kuka.jacobian(q);

            J_p = J.block(0,0,3,7);
            J_o = J.block(3,0,3,7);

            //orientamento fisso e pari a quello iniziale
            Euler_d = kuka.Rot2Euler(R_e);
            Euler_e = kuka.Rot2Euler(R_e);
            T = kuka.T_euler(Euler_e);

            J_o = T.inverse()*J_o;

            Ja.block(0,0,3,7) = J_p;
            Ja.block(3,0,3,7) = J_o;


            
           
            J_inv = (Ja.transpose())*(Ja*Ja.transpose()).inverse();
            w_now = kuka.manip(J);

            for (int k=0 ; k<7; k++){
                q_inc=q;
                q_inc(k) = q_inc(k)+0.01;
                J_inc = kuka.jacobian(q_inc);
                
                w_inc = kuka.manip(J_inc);
                grad_w(k)=(w_inc-w_now)/0.01;
            }
    
            //cout<< "w" <<endl;
            //cout<<w_now<<endl;
            dq0 = k0*grad_w;

            //cout <<" dq0 "<<endl;
            //cout << dq0 <<endl;
 

  
            xe(0) = Te(0,3);
            xe(1) = Te(1,3);
            xe(2) = Te(2,3);
            xe(3) = Euler_e(0);
            xe(4) = Euler_e(1);
            xe(5) = Euler_e(2);

            xd(3) = Euler_d(0);
            xd(4) = Euler_d(1);
            xd(5) = Euler_d(2);
         

            //eo = Euler_d-Euler_e;
            cout << "x" << endl;
            cout << xe << endl;


            cout << "xd" << endl;
            cout << xd << endl;

            xep = xe.head(3);
            xdp = xd.head(3);

            ep = error_p(xep, xdp);

            //cout << "ep" << endl;
            //cout << ep << endl;



            ddz=invMd*(he-Kd*dz-Kpa*z);

            dz=dz+0.01*ddz;
            z = z +0.01*dz;

            

            e = xd-z-xe;
            
            dxt = dz+dxd;

           
       


        

            cout<< "e "<< endl;
            cout << e <<endl;
            

            q_dot = J_inv*(dxt + K*e) + (Id - J_inv*Ja)*dq0;

            
            q = q + q_dot*0.01;

            


            cout<< "q "<<endl;
            cout<< q <<endl;


            // saving output files
            ofstream file1;
            file1.open("xde.txt",ios::out|ios::app);
            if(file1.is_open()){
                for(int i=0;i<3;i++){
                    file1<<ep(i);
                    file1<<" ";}
                file1<<"\n";
                file1.close();}
            else cout<<"impossibile aprire file";

            
            ofstream file2;
            file2.open("xe.txt",ios::out|ios::app);
            if(file2.is_open()){
                for(int i=0;i<3;i++){
                    file2<<xe(i);
                    file2<<" ";}
                file2<<"\n";
                file2.close();}
            else cout<<"impossibile aprire file";


            ofstream file3;
            file3.open("xd.txt",ios::out|ios::app);
            if(file3.is_open()){
                for(int i=0;i<3;i++){
                    file3<<xd(i);
                    file3<<" ";}
                file3<<"\n";
                file3.close();}
            else cout<<"impossibile aprire file";

            ofstream file4;
            file4.open("w.txt",ios::out|ios::app);
            if(file4.is_open()){
                file4<<w_now;
                file4<<"\n";
                file4.close();}
            else cout<<"impossibile aprire file";

            ofstream file5;
            file5.open("he.txt",ios::out|ios::app);
            if(file5.is_open()){
                for(int i=0;i<6;i++){
                    file5<<he(i);
                    file5<<" ";}
                    file5<<"\n";
                file5.close();}
            else cout<<"impossibile aprire file";
            

            ofstream file6;
            file6.open("e.txt",ios::out|ios::app);
            if(file6.is_open()){
                for(int i=0;i<6;i++){
                    file6<<e(i);
                    file6<<" ";}
                file6<<"\n";
                file6.close();}
            else cout<<"impossibile aprire file";


     
                
            for(int i=0; i<7; i++){
                q_cmd[i].data = q[i];
                joint_pos_cmd[i].publish(q_cmd[i]);
            }
               
            rate.sleep();
            p++;
        }
    }
    cout<<"Trajectory ended!"<<endl;

    return 0;
}
