#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <fstream>
#include <stdio.h>
#include <math.h>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 7> Matrix6d;

class kuka
{
        public:

        //constructor
        kuka();


        //forward kine
        Matrix4d Te(Vector7d q);

        //Jacobian
        Matrix6d J(Vector7d q);
        
};