//编程求解线性方程　Ax = b
//A为100 * 100的随机矩阵

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

using namespace std;
using namespace Eigen;

#define MATRIX_SIZE 100

int main( int argc, char** argv )
{
    //构造动态矩阵, 类型为double型
    MatrixXd Random_Matrix = MatrixXd::Random( MATRIX_SIZE, MATRIX_SIZE );
    MatrixXd A = Random_Matrix * Random_Matrix.transpose() ;                  //使得A为正定对称矩阵，才能使得cholesky分解成功
    VectorXd B = VectorXd::Random( MATRIX_SIZE );
    VectorXd x = A.colPivHouseholderQr().solve(B);          //调用QR分解求解
    cout <<"调用QR分解求得Ax=b方程的解为\n"<< x << endl;
	//注意这里并不能确定A为正定矩阵，只能说是半正定，因此不能直接用llt方法来求解
    VectorXd y = A.ldlt().solve(B);                          //调用cholesky分解求解
    cout <<"调用cholesky分解求得Ax=b方程的解为\n"<< y << endl;

    return 0;
}
