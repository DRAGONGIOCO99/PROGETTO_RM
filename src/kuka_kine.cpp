#include <stdio.h>
#include <math.h>
#include "/home/dev/rl_ros1/src/PROGETTO_RM/include/kuka_kine.h"

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
    DH_table(0,1)=-M_PI/2;
    DH_table(1,1)=M_PI/2;
    DH_table(2,1)=M_PI/2;
    DH_table(3,1)=-M_PI/2;
    DH_table(4,1)=-M_PI/2;
    DH_table(5,1)=M_PI/2;
    DH_table(6,1)=0;

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


Matrix6d kuka_robot::jacobian(Vector7d q){
    Matrix4d Temp = Matrix4d::Zero();
    Matrix4d A0 = Matrix4d::Zero();
    Matrix6d J = Matrix6d::Zero();
    Eigen::Matrix<double, 3, 7> p=Eigen::Matrix<double, 3, 7>::Zero();
    Eigen::Matrix<double, 3, 7> z=Eigen::Matrix<double, 3, 7>::Zero();
    Vector3d z0=Vector3d::Zero();
    Vector3d p0=Vector3d::Zero();

    z0(2)=1;

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

    for (int i=0; i<7 ;i++){
        
        p(0,i)=A0(0,3); // estraggo vettore p_i
        p(1,i)=A0(1,3);
        p(2,i)=A0(2,3);

        z(0,i)=A0(0,2); // estraggo vettore z_i
        z(1,i)=A0(1,2);
        z(2,i)=A0(2,2);
    
        Temp=Homogeneous[i+1];
        A0= A0*Temp;
    }

J.col(0).segment<3>(0)=z0.cross(p.col(6)-p0);
J.col(0).segment<3>(3)=z0;

    for (int i=1; i<7 ;i++){
        
        J.col(i).segment<3>(0)=(z.col(i-1)).cross(p.col(6)-p.col(i-1));
        J.col(i).segment<3>(3)=z.col(i-1);
    
    }

return J;


}

Vector4d kuka_robot::Rot2Quat(Matrix3d R){
    Vector4d Q = Vector4d::Zero();



    double argument1 = R(0,0) - R(1,1) - R(2,2) + 1.0;
    double argument2 = -R(0,0) + R(1,1) - R(2,2) + 1.0;
    double argument3 = -R(0,0) - R(1,1) + R(2,2) + 1.0;

    if(argument1 < 0.0 && argument1 > -pow(10,-12)){
   //     cout<<"WARNING: argument1 in function 'rot2quat' is negative and near zero: -10^(-12) < argument1 < 0"<<endl;
        argument1 = 0.0;
    }
    else if(argument1 < -pow(10,-12)){
     //   cout<<"WARNING: argument1 in function 'rot2quat' is negative and NOT near zero: argument1 < -10^(-12)"<<endl;
        argument1 = 0.0;
    }
    if(argument2 < 0.0 && argument2 > -pow(10,-12)){
       // cout<<"WARNING: argument2 in function 'rot2quat' is negative and near zero: -10^(-12) < argument2 < 0"<<endl;
        argument2 = 0.0;
    }
    else if(argument2 < -pow(10,-12)){
        //cout<<"WARNING: argument2 in function 'rot2quat' is negative and NOT near zero: argument2 < -10^(-12)"<<endl;
        argument2 = 0.0;
    }
    if(argument3 < 0.0 && argument3 > -pow(10,-12)){
      //  cout<<"WARNING: argument3 in function 'rot2quat' is negative and near zero: -10^(-12) < argument3 < 0"<<endl;
        argument3 = 0.0;
    }
    else if(argument3 < -pow(10,-12)){
      //  cout<<"WARNING: argument3 in function 'rot2quat' is negative and NOT near zero: argument3 < -10^(-12)"<<endl;
        argument3 = 0.0;
    }


    Q(3) = 0.5*sqrt(R(0,0) + R(1,1) + R(2,2) +1.0);

    if(isnan(Q(3)))
   
   
    Q(3) = 0.0;

    if((R(2,1) - R(1,2)) >= 0)
        Q(0) = 0.5*sqrt(argument1);
    else
        Q(0) = -0.5*sqrt(argument1);

    if((R(0,2) - R(2,0))>= 0)
        Q(1) = 0.5*sqrt(argument2);
    else
        Q(1) = -0.5*sqrt(argument2);

    if ((R(1,0) - R(0,1)) >= 0)
        Q(2) = 0.5*sqrt(argument3);
    else
        Q(2) = -0.5*sqrt(argument3);

   
    return Q;

}

Vector3d kuka_robot::QuatError(Vector4d Qd, Vector4d Qe){

        Vector3d ed_app = Vector3d::Zero();
        Vector3d e_app = Vector3d::Zero();
        Vector3d eo = Vector3d::Zero();

        for(int i=0; i<3; i++){
            ed_app(i) = Qd(i);
            e_app(i) = Qe(i);
        }
        
        
	    double etatilde = Qe(3)*Qd(3) +e_app.transpose() * ed_app;
        eo = Qe(3)*ed_app - Qd(3)*e_app - ed_app.cross(e_app);
        double angle = atan2(eo.norm(), etatilde);
	
	    if(angle > M_PI/2){
		
		eo = -eo;
		
	    }

    
        return eo;


}
