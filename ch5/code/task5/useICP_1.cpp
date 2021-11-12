#include <sophus/se3.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace std;
using namespace Eigen;
using namespace cv;
// path to trajectory file
string trajectory_file = "../compare.txt";
// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void icp_3d(const vector<Point3f>& pts1,const vector<Point3f>& pts2,Matrix3d& R,Vector3d &t);
void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses_e,vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses_g);

int main(int argc, char **argv) {

    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses_e;
    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses_g;
    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses_g_t;
    Matrix3d R;
    Vector3d t;
    vector<Point3f> pts_e,pts_g;
    ifstream fin(trajectory_file);
    if(!fin)
    {
        cout<<"can't find file at "<<trajectory_file<<endl;
        return 1;
    }
    while(!fin.eof())
    {
        double t1,tx1,ty1,tz1,qx1,qy1,qz1,qw1;
        double t2,tx2,ty2,tz2,qx2,qy2,qz2,qw2;
        fin>>t1>>tx1>>ty1>>tz1>>qx1>>qy1>>qz1>>qw1>>t2>>tx2>>ty2>>tz2>>qx2>>qy2>>qz2>>qw2;
        pts_e.push_back(Point3f(tx1,ty1,tz1));
        pts_g.push_back(Point3f(tx2,ty2,tz2));
        Quaterniond q1(qw1,qx1,qy1,qz1);
        Quaterniond q2(qw2,qx2,qy2,qz2);
        Vector3d v1(tx1,ty1,tz1);
        Vector3d v2(tx2,ty2,tz2);
        Sophus::SE3d SE3_qt1(q1,v1);
        Sophus::SE3d SE3_qt2(q2,v2);
        poses_e.push_back(SE3_qt1);
        poses_g_t.push_back(SE3_qt2);
    }
    cout<<"read total "<<poses_e.size()<<"pose entries"<<endl;
    cout<<"read total "<<poses_g_t.size()<<"pose entries"<<endl;
    icp_3d(pts_e,pts_g,R,t);
    Sophus::SE3d T_eg(R,t);
    for(auto SE_g:poses_g_t)
    {
        Sophus::SE3d T_e=T_eg*SE_g;
        poses_g.push_back(T_e);
    }
    // draw trajectory in pangolin
    DrawTrajectory(poses_e,poses_g);
    return 0;
}
void icp_3d(const vector<Point3f>& pts1,const vector<Point3f>& pts2,Matrix3d& R,Vector3d &t)
{
    //求解质心
    Point3f p1,p2;
    int n=pts1.size();
    for(int i=0;i<n;i++)
    {
        p1+=pts1[i];
        p2+=pts2[i];
    }
    p1=Point3f(Vec3f(p1)/n);
    p2=Point3f(Vec3f(p2)/n);
    //去质心坐标

    vector<Point3f> q1(n),q2(n);
    for(int i=0;i<n;i++)
    {
        q1[i]=pts1[i]-p1;
        q2[i]=pts2[i]-p2;
    }
    //定义与去q1,q2有关的矩阵W
    Matrix3d W=Matrix3d::Zero();
    for(int i=0;i<n;i++)
    {
        W+=Vector3d(q1[i].x,q1[i].y,q1[i].z)*Vector3d(q2[i].x,q2[i].y,q2[i].z).transpose();
    }
    cout<<"W=\n"<<W<<endl;
    //SVD分解W
    JacobiSVD<MatrixXd> svd(W,ComputeThinU | ComputeThinV);
    Matrix3d U=svd.matrixU();
    Matrix3d V=svd.matrixV();
    cout<<"U=\n"<<U<<endl;
    cout<<"V=\n"<<V<<endl;
    //R=U×V^T
    R=U*V.transpose();
    //t=p-Rp'
    t=Vector3d(p1.x,p1.y,p1.z)-R*Vector3d(p2.x,p2.y,p2.z);
}


/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses_e,vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses_g) {
    if (poses_e.empty() || poses_g.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses_e.size() - 1; i++) {
            glColor3f(1 - (float) i / poses_e.size(), 0.0f, (float) i / poses_e.size());
            glBegin(GL_LINES);
            auto p1 = poses_e[i], p2 = poses_e[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        for (size_t i = 0; i < poses_g.size() - 1; i++) {
            glColor3f(1 - (float) i / poses_g.size(), 0.0f, (float) i / poses_g.size());
            glBegin(GL_LINES);
            auto p1 = poses_g[i], p2 = poses_g[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}