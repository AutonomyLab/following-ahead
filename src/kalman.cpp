/**
* Implementation of KalmanFilter class.
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <iostream>
#include <stdexcept>

#include "kalman.hpp"
#include "utils.hpp"

/**
 * Our state(x): x, y, theta, x_dot, y_dot
 * Our control input (u): cmd_vel (v, omega)
 * Our measurement (y): odom (v, omega)
 */
KalmanFilter::KalmanFilter(
    double dt, // time between previous and current
    const cv::Mat& Q, // uncertainty in prediction model
    const cv::Mat& R, // uncertainty in measurement
    const cv::Mat& P) // error covariance of the state vector
  : Q(Q), R(R), P0(P),
    m(4), n(11), dt(dt), initialized(false),
    I(cv::Mat::eye(n, n, CV_32F)), x_hat(cv::Mat(n, 1, CV_32F)), x_hat_new(cv::Mat(n, 1, CV_32F))
{
  C = cv::Mat::zeros(m, n, CV_32F);
  C.at<float>(0, 3) = 1;
  C.at<float>(1, 4) = 1;
  C.at<float>(2, 5) = 1;
  C.at<float>(3, 6) = 1;
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const cv::Mat& x0) {
  x_hat = x0.clone();
  P = P0.clone();
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void KalmanFilter::init() {
  x_hat = cv::Mat::zeros(n, n, CV_32F);
  P = P0.clone();
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::update(const cv::Mat& y, const cv::Mat& u) {
  
  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  float cosTheta = cos(x_hat.at<float>(2, 0)); 
  float sinTheta = sin(x_hat.at<float>(2, 0));
  float x_r =  x_hat.at<float>(0, 0);
  float y_r =  x_hat.at<float>(1, 0);
  float x_p =  x_hat.at<float>(5, 0);
  float y_p =  x_hat.at<float>(6, 0);

  // x_hat_newx_hat_new.at<float>(7, 0) =  = A * x_hat;
  /*
   * Our state propagation model is nonlinear :(, cannot do matrix multiplication directly
   */
  x_hat_new.at<float>(0, 0) = x_hat.at<float>(0, 0) + x_hat.at<float>(3, 0) * cos(x_hat.at<float>(2, 0)) * dt;
  x_hat_new.at<float>(1, 0) = x_hat.at<float>(1, 0) + x_hat.at<float>(3, 0) * sin(x_hat.at<float>(2, 0)) * dt;
  x_hat_new.at<float>(2, 0) = x_hat.at<float>(2, 0) + x_hat.at<float>(4, 0) * dt;
  // no velocity smoothing prior
  // x_hat_new.at<float>(3, 0) = u.at<float>(0, 0);
  // x_hat_new.at<float>(4, 0) = u.at<float>(1, 0);
  x_hat_new.at<float>(3, 0) = (x_hat.at<float>(3, 0) + u.at<float>(0, 0))/2.0;
  x_hat_new.at<float>(4, 0) = (x_hat.at<float>(4, 0) + u.at<float>(1, 0))/2.0;
  // assume static prior on other robot position :(
  // x_hat_new.at<float>(5, 0) = x_hat.at<float>(5, 0);
  // x_hat_new.at<float>(6, 0) = x_hat.at<float>(6, 0);
  x_hat_new.at<float>(5, 0) = (x_hat.at<float>(5, 0) + x_hat.at<float>(7, 0)*dt);
  x_hat_new.at<float>(6, 0) = (x_hat.at<float>(6, 0) + x_hat.at<float>(8, 0)*dt);
  x_hat_new.at<float>(7, 0) = ((x_hat.at<float>(5, 0) -  x_hat.at<float>(9, 0))/dt + x_hat.at<float>(7, 0)) / 2;
  x_hat_new.at<float>(8, 0) = ((x_hat.at<float>(6, 0) -  x_hat.at<float>(10, 0))/dt + x_hat.at<float>(8, 0)) / 2;
  
  // previous previous person
  x_hat_new.at<float>(9, 0) = x_hat.at<float>(5, 0);
  x_hat_new.at<float>(10, 0) = x_hat.at<float>(6, 0);

  // std::cout << "Prior: " << x_hat_new.t() << std::endl;

  // reduce the states
  // cv::Mat x_red = x_hat_new.rowRange(0, 7);
  // cv::Mat A_red = A.rowRange(0, 7).colRange(0, 7);
  // cv::Mat P_red = P.rowRange(0, 7).colRange(0, 7);
  // cv::Mat C_red = C.colRange(0, 7);
  // cv::Mat Q_red = Q.rowRange(0, 7).colRange(0, 7);

  // P_red = A_red*P_red*A_red.t() + Q_red;
  // K = P_red*C_red.t()*(C_red*P_red*C_red.t() + R).inv();

  /**
   *
   * Our measurement: the absolute pose of the person depends on the relative pose given by blob detection as well as the robot pose, which is part of our state
   * Hence, the uncertainty of both the relative pose and robot pose should be included in the measurement update
   * The uncertainty in blob detection is constant ( R(2:, 2:) ) while the uncertainty in robot pose depends on the Kalman Filter state error covariance
   */
  float x_rel = y.at<float>(2, 0);
  float y_rel = y.at<float>(3, 0);

  // the jacobian of the measurement model (only absolute person position) with respect to robot pose
  cv::Mat J_r = cv::Mat::zeros(2, 3, CV_32F); // 3 for x, y and theta of robot
  J_r.at<float>(0, 0) = 1;
  J_r.at<float>(1, 1) = 1;
  J_r.at<float>(0, 2) = -x_rel * sinTheta - y_rel * cosTheta;
  J_r.at<float>(1, 2) = x_rel * cosTheta - y_rel * sinTheta;

  // std::cout << "J_r" << std::endl;

  // the jacobian of the measurement model (only absolute person position) with respect to relative person position
  cv::Mat J_rel(2, 2, CV_32F);
  J_rel.at<float>(0, 0) = cosTheta;
  J_rel.at<float>(0, 1) = -sinTheta;
  J_rel.at<float>(1, 0) = sinTheta;
  J_rel.at<float>(1, 1) = cosTheta;

  // std::cout << "J_rel" << std::endl;

  cv::Mat R_full = cv::Mat::zeros(m, m, CV_32F);
  // the ones corresponding to odometry go as it is
  R.rowRange(0, 2).colRange(0, 2).copyTo(R_full.rowRange(0, 2).colRange(0, 2));
  // std::cout << "R1" << std::endl;
  // the ones corresponding to absolute person position (sum of propagated covariance of relative position and robot pose)
  cv::Mat propagated_covariance = J_rel * R.rowRange(2, 4).colRange(2, 4) * J_rel.t() + J_r * P.rowRange(0, 3).colRange(0, 3) * J_r.t();
  propagated_covariance.copyTo(R_full.rowRange(2, 4).colRange(2, 4));
  // std::cout << "R2" << std::endl; 

  P = A*P*A.t() + Q;
  K = P*C.t()*(C*P*C.t() + R_full).inv();
  
  // std::cout << "K: " << std::endl;
  // cv::Mat h(m, 1, CV_32F);
  // h.at<float>(0, 0) = x_hat_new.at<float>(3, 0);
  // h.at<float>(1, 0) = x_hat_new.at<float>(4, 0);
  // when our measurement is relative pose
  // h.at<float>(2, 0) = cosTheta*x_p + sinTheta*y_p - (cosTheta*x_r + sinTheta * y_r) * x_p;
  // h.at<float>(3, 0) = -sinTheta*x_p + cosTheta*y_p + (sinTheta* x_r - cosTheta*y_r) * y_p;
  // when our measurement is absolute pose
  // h.at<float>(2, 0) = x_p;
  // h.at<float>(3, 0) = y_p;

  // std::cout << "y: " << y.t() << " h: " << h.t() << std::endl;
  // x_red += K * (y - h);
  // P_red = (I.rowRange(0, 7).colRange(0, 7) - K*C_red)*P_red;
  // x_red.copyTo(x_hat.rowRange(0, 7));
  // P_red.copyTo(P.rowRange(0, 7).colRange(0, 7));
  
  // the x_hat should be replaced by x_hat_new in real robot case
  cv::Mat y_homogeneous = cv::Mat::ones(3, 1, CV_32F);
  y.rowRange(2, 4).copyTo(y_homogeneous.rowRange(0, 2));
  cv::Mat y_absolute_homogeneous = xytheta2TransformationMatrix(x_hat.rowRange(0, 3)) * y_homogeneous;
  // std::cout << "y transformation" << std::endl;

  cv::Mat y_final(m, 1, CV_32F);
  y.rowRange(0, 2).copyTo(y_final.rowRange(0, 2));
  y_absolute_homogeneous.rowRange(0, 2).copyTo(y_final.rowRange(2, 4));
  
  x_hat_new += K * (y_final - C*x_hat_new);
  // std::cout << "x_hat_new" << std::endl;
  P = (I - K*C)*P;
  x_hat = x_hat_new;

  // std::cout << "Posteriori: " << x_hat.t() << std::endl;

  t += dt;
}

void KalmanFilter::update(const cv::Mat& y, double dt, const cv::Mat& u) {

  float cosTheta = cos(x_hat.at<float>(2, 0)); 
  float sinTheta = sin(x_hat.at<float>(2, 0));
  float x_r =  x_hat.at<float>(0, 0);
  float y_r =  x_hat.at<float>(1, 0);
  float x_p =  x_hat.at<float>(5, 0);
  float y_p =  x_hat.at<float>(6, 0);
  
  this->A = cv::Mat::zeros(n, n, CV_32F);
  A.at<float>(0, 0) = A.at<float>(1, 1) = A.at<float>(2, 2) = 1;
  
  // no velocity smoothing prior
  // A.at<float>(3, 3) = A.at<float>(4, 4) = 0;
  A.at<float>(3, 3) = A.at<float>(4, 4) = 0.5;

  A.at<float>(0, 2) = -x_hat.at<float>(3, 0) * sinTheta * dt;
  A.at<float>(0, 3) = cosTheta* dt;

  A.at<float>(1, 2) = x_hat.at<float>(3, 0) * cosTheta * dt;
  A.at<float>(1, 3) = sinTheta * dt;

  A.at<float>(2, 4) = dt;

  A.at<float>(5, 5) = A.at<float>(6, 6) = 1;

  A.at<float>(7, 7) = A.at<float>(8, 8) = 1/2;

  A.at<float>(5, 7) = A.at<float>(6, 8) = dt;

  A.at<float>(7, 5) = A.at<float>(8, 6) = 1.0/(2.0*dt);
  A.at<float>(7, 9) = A.at<float>(8, 10) = -1.0/(2.0*dt);


  A.at<float>(8, 5) = A.at<float>(10, 6) = 1;

  // C.at<float>(2, 0) = -cosTheta*x_p;
  // C.at<float>(2, 1) = -sinTheta*x_p;
  // C.at<float>(2, 2) = -sinTheta*x_p + cosTheta*y_p - (-sinTheta*x_r+cosTheta*y_r) * x_p;
  // C.at<float>(2, 5) = cosTheta - (cosTheta*x_r + sinTheta*y_r);
  // C.at<float>(2, 6) = sinTheta;

  // C.at<float>(3, 0) = sinTheta*y_p;
  // C.at<float>(3, 1) = -cosTheta*y_p;
  // C.at<float>(3, 2) = -cosTheta*x_p - sinTheta*y_p + (cosTheta*x_r + sinTheta*y_r)*y_p;
  // C.at<float>(3, 5) = -sinTheta;
  // C.at<float>(3, 6) = cosTheta + sinTheta*x_r - cosTheta*y_r;
  
  this->dt = dt;
  update(y, u);
}
