#include <iostream>
#include "kalman_filter.h"

KalmanFilter::KalmanFilter()
{
}

void KalmanFilter::set_fixed( MatrixXf _A, MatrixXf _H, MatrixXf _Q, MatrixXf _R, MatrixXf _B )
{
	A = _A;
	B = _B;
	H = _H;
	Q = _Q;
	R = _R;
	I = I.Identity(2, 2);
}

void KalmanFilter::set_initial( VectorXf _X0, MatrixXf _P0 ){
	X0 = _X0;
	P0 = _P0;
}

void KalmanFilter::predict( VectorXf U ){
  X = (A * X0) + (B * U);
  P = (A * P0 * A.transpose()) + Q;
}

void KalmanFilter::update( VectorXf Z ) {
  K = ( P * H.transpose() ) * ( H * P * H.transpose() + R).inverse();

  X = X + K*(Z - H * X);
  
  P = (I - K * H) * P;

  X0 = X;
  P0 = P;
}