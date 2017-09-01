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
 * Our state(x): x, y, vel, theta, x[t-1], y[t-1]
 * Our measurement (y): odom (v, omega)
 */
PersonKalman::PersonKalman(
    double dt, // time between previous and current
    const cv::Mat& Q, // uncertainty in prediction model
    const cv::Mat& R, // uncertainty in measurement
    const cv::Mat& P) // error covariance of the state vector
  : Q(Q), R(R), P0(P),
    m(3), n(7), dt(dt), initialized(false),
    I(cv::Mat::eye(n, n, CV_32F)), x_hat(cv::Mat(n, 1, CV_32F)), x_hat_new(cv::Mat(n, 1, CV_32F))
{
  C = cv::Mat::zeros(m, n, CV_32F);
  C.at<float>(X_T_IDX, X_T_IDX) = 1;
  C.at<float>(Y_T_IDX, Y_T_IDX) = 1;
  C.at<float>(2, THETA_IDX) = 1;
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

  bool is_omega_non_zero = true;

  float theta = x_hat.at<float>(THETA_IDX, 0);
  float cosTheta = cos(theta); 
  float sinTheta = sin(theta);
  float omega = x_hat.at<float>(OMEGA_IDX, 0);
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

  float new_theta = atan2(del_y, del_x);

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
  
  float theta_hat_new = theta;
  if (omega > 1e-4)
  {
    theta_hat_new += omega * dt;
  }

  if (d > DISTANCE_EPSILON)
  {
    // this doesn't work when the angle is around 180 (some will be say 179 and some -179 and they'll cancel to 0)
    // x_hat_new.at<float>(THETA_IDX, 0) =  atan2(del_y, del_x) * THETA_IIR_ALPHA + theta * (1 - THETA_IIR_ALPHA);
    
    cv::Point2f unit_vector_new_theta(cos(new_theta), sin(new_theta));
    cv::Point2f unit_vector_theta(cos(theta), sin(theta));

    // signed angle between two vectors
    // |A·B| = |A| |B| COS(θ)
    // |A×B| = |A| |B| SIN(θ)
    // Math.Atan2(Cross(A,B), Dot(A,B));
    // reference https://stackoverflow.com/questions/21483999/using-atan2-to-find-angle-between-two-vectors
    float cross = unit_vector_theta.x * unit_vector_new_theta.y - unit_vector_theta.y * unit_vector_new_theta.x;
    float del_theta = atan2(cross, unit_vector_new_theta.dot(unit_vector_theta));

    // if (del_theta < 1e-5)
    // {
    //   ROS_ERROR("del_theta low, omega 0");
    //   x_hat_new.at<float>(OMEGA_IDX, 0) = 0.0;
    //   is_omega_non_zero = false;
    // }
    // else
    {  
      ROS_ERROR("%f - %f = %f", new_theta * 180 / M_PI, theta * 180 / M_PI, del_theta);
      x_hat_new.at<float>(OMEGA_IDX, 0) = (OMEGA_IIR_ALPHA * del_theta / dt) + (1 - OMEGA_IIR_ALPHA) * omega;
    }
    // ROS_WARN("new theta:%f old theta:%f average:%f",new_theta, theta, x_hat_new.at<float>(THETA_IDX, 0));
  }
  else
  {
    ROS_ERROR("distance less, so omega 0");
    x_hat_new.at<float>(OMEGA_IDX, 0) = 0.0; // omega; 
    is_omega_non_zero = false;
  }

  if (!is_omega_non_zero)
  {
    ROS_ERROR("Zero omega");
    x_hat_new.at<float>(OMEGA_IDX, 0) = 0;
  }

  x_hat_new.at<float>(THETA_IDX, 0) = atan2(sin(theta_hat_new), cos(theta_hat_new));

  cv::Mat augmented_y(3, 1, CV_32F);
  y.copyTo(augmented_y.rowRange(0, 2));

  // the angle is new_theta (averaged)  
  // if (is_omega_non_zero)
  {
    cv::Point2f unit_vector1(cos(new_theta), sin(new_theta));
    cv::Point2f unit_vector2(cos(theta), sin(theta));
    cv::Point2f sum_vector = unit_vector1 * THETA_IIR_ALPHA + unit_vector2 * (1 - THETA_IIR_ALPHA);
    new_theta = atan2(sum_vector.y, sum_vector.x);
  }
  // else
  // {
  //   new_theta = theta;
  // }
  augmented_y.at<float>(2, 0) = new_theta;

  P = A*P*A.t() + Q;
  K = P*C.t()*(C*P*C.t() + R).inv();
  
  x_hat_new += K * (augmented_y - C*x_hat_new);
  P = (I - K*C)*P;
  x_hat = x_hat_new;

  // if (!is_omega_non_zero)
  // {
  //   x_hat.at<float>(OMEGA_IDX, 0) = 0;
  // }

  t += dt;
}

void PersonKalman::update(const cv::Mat& y, double dt, cv::Mat R) {
  if (R.rows == m && R.cols == m) 
  {
    this->R = R.clone();
  }
  else
  {
    ROS_ERROR("R is not mxm");
  }

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
    A.at<float>(THETA_IDX, THETA_IDX) = 1;
    A.at<float>(THETA_IDX, OMEGA_IDX) = dt;
  }
  else
  {
    // just propagating last state
    A.at<float>(VEL_IDX, VEL_IDX) = 1;
  }

  if ( fabs(d) > DISTANCE_EPSILON ) 
  {
    A.at<float>(OMEGA_IDX, X_T_IDX) = - OMEGA_IIR_ALPHA * del_y / d_2 / dt;
    A.at<float>(OMEGA_IDX, Y_T_IDX) = OMEGA_IIR_ALPHA * del_x / d_2 / dt;
    A.at<float>(OMEGA_IDX, X_T_1_IDX) = OMEGA_IIR_ALPHA * del_y / d_2 / dt;
    A.at<float>(OMEGA_IDX, Y_T_1_IDX) = - OMEGA_IIR_ALPHA * del_x / d_2 / dt;

    A.at<float>(THETA_IDX, OMEGA_IDX) = - OMEGA_IIR_ALPHA / dt;
    A.at<float>(OMEGA_IDX, OMEGA_IDX) = 1 - OMEGA_IIR_ALPHA;
  }
  else
  {
    // just propagating last state
    A.at<float>(OMEGA_IDX, OMEGA_IDX) = 1;
  }
  this->dt = dt;
  update(y);
}
