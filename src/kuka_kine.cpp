#include <stdio.h>
#include <math.h>
#include "/home/dev/ros1_ws/src/PROGETTO_RM/include/kuka_kine.h"

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

kuka_robot::kuka_robot(){
    // prima colonna a
    DH_table(0,0)=0;
    DH_table(1,0)=0;
    DH_table(2,0)=0;
    DH_table(3,0)=0;
    DH_table(4,0)=0;
    DH_table(5,0)=0;
    DH_table(6,0)=0;

    //seconda colonna alpha
    DH_table(0,1)=0;
    DH_table(1,1)=-M_PI/2;
    DH_table(2,1)=M_PI/2;
    DH_table(3,1)=M_PI/2;
    DH_table(4,1)=-M_PI/2;
    DH_table(5,1)=-M_PI/2;
    DH_table(6,1)=M_PI/2;

    // terza colonna d
    DH_table(0,2)=0.34;
    DH_table(1,2)=0;
    DH_table(2,2)=0.4;
    DH_table(3,2)=0;
    DH_table(4,2)=0.4;
    DH_table(5,2)=0;
    DH_table(6,2)=0.126;


    // quarta colonna theta
    DH_table(0,3)=0;
    DH_table(1,3)=0;
    DH_table(2,3)=0;
    DH_table(3,3)=0;
    DH_table(4,3)=0;
    DH_table(5,3)=0;
    DH_table(6,3)=0;


    
}



Matrix4d kuka_robot::Te(Vector7d q){
//qua bisogna scirvere la cinematica diretta
    Matrix4d A0 = Matrix4d::Zero();
    Matrix4d Temp = Matrix4d::Zero();
    
    //cout<<q.size()<<endl;

    

    double q1 = q[0];
    double q2 = q[1];
    double q3 = q[2];
    double q4 = q[3];
    double q5 = q[4];
    double q6 = q[5];
    double q7 = q[6];

    DH_table(0,3)=q1;
    DH_table(1,3)=q2;
    DH_table(2,3)=q3;
    DH_table(3,3)=q4;
    DH_table(4,3)=q5;
    DH_table(5,3)=q6;
    DH_table(6,3)=q7;

    
    //Eigen::Tensor<double, 3> Homogeneous(4, 4, 7);
    std::vector<Matrix4d> Homogeneous(7);  // Vettore di matrici di dimensione 3x3

    for (int i=0; i<7 ;i++){
        Homogeneous[i](0,0) = cos(DH_table(i,3));
        Homogeneous[i](0,1)  = -sin(DH_table(i,3))*cos(DH_table(i,1));
        Homogeneous[i](0,2)  = sin(DH_table(i,3))*sin(DH_table(i,1));;
        Homogeneous[i](0,3)  = DH_table(i,0)*cos(DH_table(i,3));
        Homogeneous[i](1,0)  = sin(DH_table(i,3));
        Homogeneous[i](1,1)  = cos(DH_table(i,3))*cos(DH_table(i,1));
        Homogeneous[i](1,2)  = -cos(DH_table(i,3))*sin(DH_table(i,1));
        Homogeneous[i](1,3)  = DH_table(i,0)*sin(DH_table(i,3));
        Homogeneous[i](2,0)  = 0;
        Homogeneous[i](2,1)  = sin(DH_table(i,1));
        Homogeneous[i](2,2)  = cos(DH_table(i,1));
        Homogeneous[i](2,3)  = DH_table(i,2);
        Homogeneous[i](3,0)  = 0;
        Homogeneous[i](3,1)  = 0;
        Homogeneous[i](3,2)  = 0;
        Homogeneous[i](3,3)  = 1;

    }

        

        A0=Homogeneous[0];

    for (int i=1; i<7 ;i++){
        Temp=Homogeneous[i];
        A0= A0*Temp;
    }

    return A0;

}


Matrix6d kuka_robot::J(Vector7d q){

//qua si calcola lo jacobiano




}
