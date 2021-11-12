#include <iostream>
#include <fstream>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>
#include <string>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;

string trajectory_file = "../compare.txt";

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;

void pose_estimation_3d3d(const vector<Point3f> &pts1,
                          const vector<Point3f> &pts2,
                          Eigen::Matrix3d &R_, Eigen::Vector3d &t_);

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti);

TrajectoryType ReadTrajectory(const string &path,int flag,vector<Point3f> &pts);

int main(int argc, char **argv) {
    vector<Point3f> pts_e,pts_g;
    //定义误差轨迹和真实轨迹
    TrajectoryType poses_e = ReadTrajectory(trajectory_file, 0, pts_e);
    TrajectoryType poses_g = ReadTrajectory(trajectory_file, 1, pts_g);
    TrajectoryType poses_gt;

    assert(!groundtruth.empty() && !estimated.empty());
    assert(groundtruth.size() == estimated.size());

    Matrix3d R;
    Vector3d t;
    // compute icp
    pose_estimation_3d3d(pts_e,pts_g,R,t);
    Sophus::SE3d T_eg(R,t);
    for(auto SE_g:poses_g)    {
        Sophus::SE3d T_e=T_eg*SE_g;
        poses_gt.push_back(T_e);
    }
    DrawTrajectory(poses_e,poses_g);
    DrawTrajectory(poses_e,poses_gt);
    return 0;
}

TrajectoryType ReadTrajectory(const string &path, int flag, vector<Point3f> &pts) {
    ifstream fin(path);
    TrajectoryType trajectory;
    if (!fin) {
        cerr << "trajectory " << path << " not found." << endl;
        return trajectory;
    }

    while (!fin.eof()) {
        double t1,tx1,ty1,tz1,qx1,qy1,qz1,qw1;
        double t2,tx2,ty2,tz2,qx2,qy2,qz2,qw2;

        fin>>t1>>tx1>>ty1>>tz1>>qx1>>qy1>>qz1>>qw1>>t2>>tx2>>ty2>>tz2>>qx2>>qy2>>qz2>>qw2;

        if(flag == 0){
            //读取误差轨迹
            Sophus::SE3d p(Eigen::Quaterniond(qw1, qx1, qy1, qz1), Eigen::Vector3d(tx1, ty1, tz1));
            pts.push_back(Point3f(tx1,ty1,tz1));
            trajectory.push_back(p);
        }else{
            Sophus::SE3d p(Eigen::Quaterniond(qw2, qx2, qy2, qz2), Eigen::Vector3d(tx2, ty2, tz2));
            pts.push_back(Point3f(tx2,ty2,tz2));
            trajectory.push_back(p);
        }
    }
    return trajectory;
}

void pose_estimation_3d3d(const vector<Point3f> &pts1,
                          const vector<Point3f> &pts2,
                          Eigen::Matrix3d &R_, Eigen::Vector3d &t_) {
    Point3f p1, p2;     // center of mass
    int N = pts1.size();
    for (int i = 0; i < N; i++) {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = Point3f(Vec3f(p1) / N);
    p2 = Point3f(Vec3f(p2) / N);
    vector<Point3f> q1(N), q2(N); // remove the center
    for (int i = 0; i < N; i++) {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++) {
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    cout << "W=" << W << endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    cout << "U=" << U << endl;
    cout << "V=" << V << endl;

    R_ = U * (V.transpose());
    if (R_.determinant() < 0) {
        R_ = -R_;
    }
    t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);
}

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti) {
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
        for (size_t i = 0; i < gt.size() - 1; i++) {
            glColor3f(0.0f, 0.0f, 1.0f);  // blue for ground truth
            glBegin(GL_LINES);
            auto p1 = gt[i], p2 = gt[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        for (size_t i = 0; i < esti.size() - 1; i++) {
            glColor3f(1.0f, 0.0f, 0.0f);  // red for estimated
            glBegin(GL_LINES);
            auto p1 = esti[i], p2 = esti[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}