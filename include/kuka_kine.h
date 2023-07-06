#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <fstream>
#include <stdio.h>
#include <math.h>
#include <Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <eigen_conversions/eigen_msg.h>
#include <vector>
using namespace Eigen;
using namespace std;

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 7> Matrix6d;
typedef Eigen::Matrix<double, 7, 4> Matrix_7_4;


class kuka_robot
{
        
        private:
        Matrix_7_4 DH_table = Matrix_7_4::Zero();
        
        
        
        
        
        public:

        //constructor
        kuka_robot();


        //forward kine
        Matrix4d Te(Vector7d q);

        //Jacobian
        Matrix6d J(Vector7d q);
        
};