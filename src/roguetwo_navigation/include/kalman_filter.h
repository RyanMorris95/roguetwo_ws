#ifndef ROUGETWO_WS_KALMAN_FILTER_H
#define ROUGETWO_WS_KALMAN_FILTER_h

#include <Eigen/Dense>

using namespace Eigen;

/*
*	Matrix Dimension must be:
*
*	A: n x n
*	B: n x m
*	H: n x n
*	Q: n x n
*	R: n x n
*	I: n x n
*	X: n x 1
*	U: m x 1
*	Z: n x 1
*	P: n x n
*	K: n x n
*
*/

class KalmanFilter 
{
public:
    //int n;  // state vector dimension
    //int m;  // control vector dimension
    KalmanFilter();

    /* Fixed Matrix */
    MatrixXf A; // system dynamics matrix
    MatrixXf B; // control matrix
    MatrixXf H; // measurement adaptation matrix
    MatrixXf Q; // process noise covariance matrix
    MatrixXf R; // measurement noise covariance matrix
    MatrixXf I; // identity matrix

    /* Variable Matrix */
    VectorXf X; // current state vector
    MatrixXf P; // state covariance
    MatrixXf K; // kalman gain matrix

    /* Initialized Value */
    VectorXf X0; // initial state vector
    MatrixXf P0; // initial staate covariance matrix

    //KalmanFilter(int _n, int _m);

    void set_fixed(MatrixXf _A, MatrixXf _B, MatrixXf _H, MatrixXf _Q, MatrixXf _R);

    void set_initial(VectorXf _X0, MatrixXf _P0);

    void predict(VectorXf U);

    void update(VectorXf Z);
};

#endif