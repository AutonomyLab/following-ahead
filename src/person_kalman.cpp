/**
* Implementation of PersonKalman class.
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <iostream>
#include <stdexcept>

#include "person_kalman.hpp"
#include "utils.hpp"


/**
 * Our state(x): x, y, theta, x_dot, y_dot
 * Our measurement (y): odom (v, omega)
 */
PersonKalman::PersonKalman(
    double dt, // time between previous and current
    const cv::Mat& Q, // uncertainty in prediction model
    const cv::Mat& R, // uncertainty in measurement
    const cv::Mat& P) // error covariance of the state vector
  : Q(Q), R(R), P0(P),
    m(2), n(6), dt(dt), initialized(false),
    I(cv::Mat::eye(n, n, CV_32F)), x_hat(cv::Mat(n, 1, CV_32F)), x_hat_new(cv::Mat(n, 1, CV_32F))
{
  C = cv::Mat::zeros(m, n, CV_32F);
  C.at<float>(X_T_IDX, X_T_IDX) = 1;
  C.at<float>(Y_T_IDX, Y_T_IDX) = 1;
}

PersonKalman::PersonKalman() {}

void PersonKalman::init(double t0, const cv::Mat& x0) {
  x_hat = x0.clone();
  P = P0.clone();
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void PersonKalman::init() {
  x_hat = cv::Mat::zeros(n, n, CV_32F);
  P = P0.clone();
  t0 = 0;
  t = t0;
  initialized = true;
}

void PersonKalman::update(const cv::Mat& y) {
  
  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  float theta = x_hat.at<float>(THETA_IDX, 0);
  float cosTheta = cos(theta); 
  float sinTheta = sin(theta);
  float vel =  x_hat.at<float>(VEL_IDX, 0);
  float x_t = x_hat.at<float>(X_T_IDX, 0);
  float y_t = x_hat.at<float>(Y_T_IDX, 0);
  float x_t_1 = x_hat.at<float>(X_T_1_IDX, 0);
  float y_t_1 = x_hat.at<float>(Y_T_1_IDX, 0);
  
  float del_x = x_t - x_t_1;
  float del_y = y_t - y_t_1;
  float d = sqrt( pow(del_x, 2) + pow(del_y, 2) );
  float m = (del_y) / (del_x);
  float m_2 = pow(m, 2);

  // x_hat_newx_hat_new.at<float>(7, 0) =  = A * x_hat;
  /*
   * Our state propagation model is nonlinear :(, cannot do matrix multiplication directly
   */
  x_hat_new.at<float>(X_T_IDX, 0) = x_t + vel * cosTheta * dt;
  x_hat_new.at<float>(Y_T_IDX, 0) = y_t + vel * sinTheta * dt;
  x_hat_new.at<float>(X_T_1_IDX, 0) = x_t;
  x_hat_new.at<float>(Y_T_1_IDX, 0) = y_t;
  // to prevent numerical issues at jacobian calculation, when the displacement is too small, just propagate last states
  x_hat_new.at<float>(VEL_IDX, 0) = d > DISTANCE_EPSILON ? d / dt * VEL_IIR_ALPHA + vel * (1 - VEL_IIR_ALPHA)
                                                      : vel;
  x_hat_new.at<float>(THETA_IDX, 0) = d > DISTANCE_EPSILON ? atan2(del_y, del_x) * THETA_IIR_ALPHA + theta * (1 - THETA_IIR_ALPHA)
                                                        : theta;

  std::cout << "A: \n" << A << std::endl;
  std::cout << "Measurement: " << y.t() << std::endl;
  std::cout << "Prior: " << x_hat_new.t() << std::endl;

  P = A*P*A.t() + Q;
  K = P*C.t()*(C*P*C.t() + R).inv();
  
  x_hat_new += K * (y - C*x_hat_new);
  std::cout << "Posterior: " << x_hat_new.t() << std::endl;
  P = (I - K*C)*P;
  x_hat = x_hat_new;

  // std::cout << "Posteriori: " << x_hat.t() << std::endl;

  t += dt;
}

void PersonKalman::update(const cv::Mat& y, double dt) {

  float cosTheta = cos(x_hat.at<float>(THETA_IDX, 0)); 
  float sinTheta = sin(x_hat.at<float>(THETA_IDX, 0));
  float vel =  x_hat.at<float>(VEL_IDX, 0);
  float x_t = x_hat.at<float>(X_T_IDX, 0);
  float y_t = x_hat.at<float>(Y_T_IDX, 0);
  float x_t_1 = x_hat.at<float>(X_T_1_IDX, 0);
  float y_t_1 = x_hat.at<float>(Y_T_1_IDX, 0);
  
  float del_x = x_t - x_t_1;
  float del_y = y_t - y_t_1;
  float del_x_2 = pow(del_x, 2);
  float del_y_2 = pow(del_y, 2);
  float d_2 = pow(del_x, 2) + pow(del_y, 2);
  float d = sqrt(d_2);
  
  this->A = cv::Mat::zeros(n, n, CV_32F);
  A.at<float>(X_T_IDX, X_T_IDX) = A.at<float>(Y_T_IDX, Y_T_IDX)
                                = A.at<float>(X_T_1_IDX, X_T_IDX) 
                                = A.at<float>(Y_T_1_IDX, Y_T_IDX) 
                                = 1;
  
  A.at<float>(X_T_IDX, VEL_IDX) = cosTheta * dt;
  A.at<float>(X_T_IDX, THETA_IDX) = -vel * sinTheta * dt;

  A.at<float>(Y_T_IDX, VEL_IDX) = sinTheta * dt;
  A.at<float>(Y_T_IDX, THETA_IDX) = vel * cosTheta * dt;

  if ( fabs(d) > DISTANCE_EPSILON )
  {
    A.at<float>(VEL_IDX, X_T_IDX) = VEL_IIR_ALPHA * del_x / d / dt;
    A.at<float>(VEL_IDX, Y_T_IDX) = VEL_IIR_ALPHA * del_y / d / dt;
    A.at<float>(VEL_IDX, X_T_1_IDX) = - VEL_IIR_ALPHA * del_x / d / dt;
    A.at<float>(VEL_IDX, Y_T_1_IDX) = - VEL_IIR_ALPHA * del_y / d / dt;

    A.at<float>(VEL_IDX, VEL_IDX) = 1 - VEL_IIR_ALPHA;
  }
  else
  {
    // just propagating last state
    A.at<float>(VEL_IDX, VEL_IDX) = 1;
  }

  if ( fabs(d_2) > DISTANCE_EPSILON ) 
  {
    A.at<float>(THETA_IDX, X_T_IDX) = - THETA_IIR_ALPHA * del_y / d_2;
    A.at<float>(THETA_IDX, Y_T_IDX) = THETA_IIR_ALPHA * del_x / d_2;
    A.at<float>(THETA_IDX, X_T_1_IDX) = THETA_IIR_ALPHA * del_y / d_2;
    A.at<float>(THETA_IDX, Y_T_1_IDX) = - THETA_IIR_ALPHA * del_x / d_2;

    A.at<float>(THETA_IDX, THETA_IDX) = 1 - THETA_IIR_ALPHA;
  }
  else
  {
    // just propagating last state
    A.at<float>(THETA_IDX, THETA_IDX) = 1;
  }
  this->dt = dt;
  update(y);
}
