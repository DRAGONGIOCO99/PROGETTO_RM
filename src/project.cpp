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

/*Vector7d dq0_limits(Vector7d &q, Vector7d &q_max, Vector7d &q_min)
{
    Vector7d w = Vector7d::Zero();

    for (unsigned i=0; i<7; i++){
        w[i] = 2*q(i) / ((q_max(i) - q_min(i))*(q_max(i) - q_min(i)));
    }

    return w;

}*/



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
    //std::vector<Vector3d> wd(n_rigXD);
    //std::vector<Matrix3d> Rd(n_rigXD);
    
    
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
    //Matrix3d R_d = Matrix3d::Zero();

    Matrix3d R_e = Matrix3d::Zero();
    //Vector3d w_d = Vector3d::Zero();
    Vector6d xe = Vector6d::Zero();
    Vector3d ep = Vector3d::Zero();
    Vector3d eo = Vector3d::Zero();

    Vector6d z = Vector6d::Zero();
    Vector6d dz = Vector6d::Zero();
    Vector6d ddz = Vector6d::Zero();


    Vector3d Euler_d = Vector3d::Zero();
    Vector3d Euler_e = Vector3d::Zero();
    Vector6d e = Vector6d::Zero();
   
    //Vector4d Q_d = Vector4d::Zero();
    //Vector4d Q_e = Vector4d::Zero();
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
    

    Vector6d he = Vector6d::Zero();

    // Forza costante
    he<<0,5,0,0,0,0;

    // Torque costante
    //he<<0,0,0,0,2,0;


    // Parametri controllo ammettenza

    Eigen::Matrix<double, 6, 6> Md = MatrixXd::Identity(6,6);
    //Md(4,4)=50;
    //DiagonalMatrix<float,3,3> Md;
   
    
    Matrix<double,6,6> invMd;
    invMd = Md.inverse();
    
    
    Eigen::Matrix<double, 6, 6> Kd = MatrixXd::Identity(6,6);
    Kd(0,0)=10;
    Kd(1,1)=10;
    Kd(2,2)=10;
    Kd(3,3)=10;
    Kd(4,4)=10;
    Kd(5,5)=10;

    Eigen::Matrix<double, 6, 6> Kpa = MatrixXd::Identity(6,6);
    Kpa(0,0)=10;
    Kpa(1,1)=100;
    Kpa(2,2)=10;
    Kpa(3,3)=10;
    Kpa(4,4)=10;
    Kpa(5,5)=10;

    float w_inc = 0.0;
    float w_now =0.0;

    double Kp = 20;
    
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

    ifstream file1("/home/dev/ros1_ws/src/PROGETTO_RM/src/Xd.txt",ios::in);
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


ifstream file2("/home/dev/ros1_ws/src/PROGETTO_RM/src/Xd_dot.txt",ios::in);
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
    
    /*
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
    */

    /*
    
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
    
    */

    /*
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

    */

    /*

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

    */
  
 /*
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
*/


    kuka_robot kuka; 
    //Te = kuka.Te(q_data);

    //R_e = Te.block(0,0,3,3);
    //cout<< "Re "<<endl;
    //cout<< R_e<<endl;

    //Euler = kuka.Rot2Euler(R_e);

    //cout<< "Euler" << endl;
    //cout<< Euler <<endl;


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
            //he(0)=20*sin(p/0.01);

            /*
            R_d = Rd[p];
            cout << "R_d" <<endl;
            cout << R_d <<endl;
            w_d(0) = wd[p](0);
            w_d(1) = wd[p](1);
            w_d(2) = wd[p](2);
            */
            
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


            
            //J_p_inv = (J_p.transpose())*(J_p*J_p.transpose()).inverse();
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

            cout <<" dq0 "<<endl;
            cout << dq0 <<endl;
            /*cout << "Te" << endl;
            cout << Te << endl;
            cout << "Jacobian" << endl;
            cout << J << endl;*/


            //Ted = kuka.Te(q_data);
            //xd <<0.5,0,0.5;
            //xe = Te.block(0,3,3,1);
            xe(0) = Te(0,3);
            xe(1) = Te(1,3);
            xe(2) = Te(2,3);
            xe(3) = Euler_e(0);
            xe(4) = Euler_e(1);
            xe(5) = Euler_e(2);

            xd(3) = Euler_d(0);
            xd(4) = Euler_d(1);
            xd(5) = Euler_d(2);
            //cout << "R_e" <<endl;
            //cout << R_e <<endl;

            
            // Quaternion extraction
           // Q_d = kuka.Rot2Quat(R_e);
            //Q_e = kuka.Rot2Quat(R_e);

            //eo=kuka.QuatError(Q_d, Q_e);
            

            //eo = Euler_d-Euler_e;
            cout << "x" << endl;
            cout << xe << endl;

            /*cout << "TE " << endl;
            cout << Te <<endl;
            cout << " Re "<<endl;
            cout << Te.block(0,0,3,3) <<endl;*/

            cout << "xd" << endl;
            cout << xd << endl;

            //ep = error_p(xe, xd);

            //cout << "ep" << endl;
            //cout << ep << endl;



            ddz=invMd*(he-Kd*dz-Kpa*z);

            dz=dz+0.01*ddz;
            z = z +0.01*dz;

            e = xd-z-xe;
            
            dxt = dz+dxd;


            /*
            cout << "\neo"<<endl;
            cout << eo << endl; 
          


            /*
            e(0) = ep(0);
            e(1) = ep(1);
            e(2) = ep(2);
            
            e(3) = eo(0);
            e(4) = eo(1);
            e(5) = eo(2);
            */

            cout<< "e "<< endl;
            cout << e <<endl;
            


            //q_dot = J_p_inv*(dxt + Kp*ep);// +(Id - J_p_inv*J_p)*dq0;
            //q_dot = J_p_inv*(dxd + Kp*ep) +(Id - J_p_inv*J_p)*dq0;
            q_dot = J_inv*(dxt + K*e) ;//+ (Id - J_inv*Ja)*dq0;

            
            q = q + q_dot*0.01;

            


            cout<< "q "<<endl;
            cout<< q <<endl;


            ofstream file1;
            file1.open("ERRORE_CLIK_pos.txt",ios::out|ios::app);
            if(file1.is_open()){
                for(int i=0;i<3;i++){
                    file1<<z(i);
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


            /*
            ofstream file2;
            file2.open("ERRORE_CLIK_or.txt",ios::out|ios::app);
            if(file2.is_open()){
                for(int i=0;i<3;i++){
                    file2<<eo(i);
                    file2<<" ";}
                file2<<"\n";
                file2.close();}
            else cout<<"impossibile aprire file";

            */
                
            for(int i=0; i<7; i++){
                q_cmd[i].data = q[i];
                joint_pos_cmd[i].publish(q_cmd[i]);
            }
               
            rate.sleep();
            p++;
        }
    }

    return 0;
}
