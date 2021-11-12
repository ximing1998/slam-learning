//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.hpp"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "../p3d.txt";
string p2d_file = "../p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    ifstream fin(p3d_file);
    if(!fin)
    {
        cout<<"不能打开文件"<<endl;
        return 1;
    }
    while(!fin.eof())
    {
        double x,y,z;
        fin>>x>>y>>z;
        Vector3d v(x,y,z);
        p3d.push_back(v);
    }
    ifstream fins(p2d_file);
    if(!fins)
    {
        cout<<"不能打开文件"<<endl;
        return 1;
    }
    while(!fins.eof())
    {
        double x,y;
        fins>>x>>y;
        Vector2d v(x,y);
        p2d.push_back(v);
    }
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;
    //vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses;
    Sophus::SE3d T_esti; // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE
            double X=p3d[i][0];
            double Y=p3d[i][1];
            double Z=p3d[i][2];
            Vector4d P_0(X,Y,Z,1);
            Vector4d P=T_esti.matrix()*P_0;
            Vector3d u=K*Vector3d(P(0,0),P(1,0),P(2,0));
            Vector2d e=p2d[i]-Vector2d(u(0,0)/u(2,0),u(1,0)/u(2,0));
            cost+=e.squaredNorm()/2;
	    // END YOUR CODE HERE

	    // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE 
            J(0,0)=fx/Z;
            J(0,1)=0;
            J(0,2)=-fx*X/(Z*Z);
            J(0,3)=-fx*X*Y/(Z*Z);
            J(0,4)=fx+fx*X*X/(Z*Z);
            J(0,5)=-fx*Y/Z;
            J(1,0)=0;
            J(1,1)=fy/Z;
            J(1,2)=-fy*Y/(Z*Z);
            J(1,3)=-fy-fy*Y*Y/(Z*Z);
            J(1,4)=fy*X*Y/(Z*Z);
            J(1,5)=fy*X/Z;
            J=-J;
	    // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

	// solve dx 
        Vector6d dx;

        // START YOUR CODE HERE 
        dx=H.ldlt().solve(b);
        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE 
        T_esti=Sophus::SE3d::exp(dx)*T_esti;
        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
