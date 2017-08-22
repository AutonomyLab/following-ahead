/**
* Kalman filter implementation using Eigen. Based on the following
* introductory paper:
*
*     http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "config.h"
#pragma once

class PersonKalman {

public:

  /**
  * Create a Kalman filter with the specified matrices.
  *   A - System dynamics matrix
  *   C - Output matrix
  *   Q - Process noise covariance
  *   R - Measurement noise covariance
  *   P - Estimate error covariance
  */
  PersonKalman(
      double dt,
      const cv::Mat& Q,
      const cv::Mat& R,
      const cv::Mat& P
  );

  /**
  * Create a blank estimator.
  */
  PersonKalman();

  /**
  * Initialize the filter with initial states as zero.
  */
  void init();

  /**
  * Initialize the filter with a guess for initial states.
  */
  void init(double t0, const cv::Mat& x0);

  /**
  * Update the estimated state based on measured values. The
  * time step is assumed to remain constant.
  */
  void update(const cv::Mat& y);

  /**
  * Update the estimated state based on measured values,
  * using the given time step and dynamics matrix.
  */
  void update(const cv::Mat& y, double dt, cv::Mat R=cv::Mat());

  /**
  * Return the current state and time.
  */
  cv::Mat state() { return x_hat; };
  double time() { return t; };

  /**
   * Is the filter initialized?
   */
  bool isInitialized() { return initialized; }

  /**
   * Reintialize the filter
   */
  void reintialize() { initialized = false; }

  cv::Mat getStateErrorCovariance() { return P; }

private:

  // Matrices for computation
  cv::Mat A, C, Q, R, P, K, P0;

  // System dimensions
  int m, n;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  cv::Mat I;

  // Estimated states
  cv::Mat x_hat, x_hat_new;
};
