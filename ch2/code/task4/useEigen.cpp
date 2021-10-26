#include <iostream>
#include <vector>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    //定义需要用到的四个四元数
    Quaterniond q_wr(0.55, 0.3, 0.2, 0.2), q_rb(0.99, 0, 0, 0.01), q_bl(0.3, 0.5, 0.2, 0.1), q_bc(0.8, 0.2, 0.1, 0.1);
    // 四元数需要进行归一化
    q_wr.normalize();
    q_rb.normalize();
    q_bl.normalize();
    q_bc.normalize();
    //定义平移向量
    Vector3d t_wr(0.1, 0.2, 0.3), t_rb(0.05, 0.0, 0.5), t_bl(0.4, 0.0, 0.5), t_bc(0.5, 0.1, 0.5);
    //相机传感器坐标系下观察到的坐标点
    Vector3d p(0.3, 0.2, 1.2);

    //欧氏变换矩阵
    Isometry3d T_WR(q_wr), T_RB(q_rb), T_BL(q_bl), T_BC(q_bc);
    T_WR.pretranslate(t_wr);
    T_RB.pretranslate(t_rb);
    T_BL.pretranslate(t_bl);
    T_BC.pretranslate(t_bc);

    //计算世界坐标系
    Vector3d p1 =  T_BL.inverse() * T_BC * p;
    cout<<"激光系下坐标:\t"<<p1.transpose()<<endl;

    //计算世界坐标系
    Vector3d p2 = T_WR * T_RB * T_BC * p;
    cout<<"世界系下坐标:\t"<<p2.transpose()<<endl;

    return 0;
}