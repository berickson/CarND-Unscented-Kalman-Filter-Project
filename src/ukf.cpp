#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

template <class T> T sign_of(T x) {
  if (x > 0)
    return T{1};
  return T{-1.};
}

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_ << 0,0,0,0,0;

  // initial covariance matrix
  P_ = MatrixXd::Constant(5, 5, 0);
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.5;     // was 30, optimized by manual parameter search
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.8; //1.0; // was 30, optimized by manual parameter search


  // wait for first measurement
  is_initialized_ = false;

  // zero timestamp is used as a special flag to say that we haven't seen a measurement yet.
  time_us_ = 0;

  // state dimension
  n_x_ = 5;

  // augmented state dimension
  n_aug_ = 7;

  // sigma point spreading parameter
  lambda_ = 3 - n_x_;


  //set vector for weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i=1; i<2*n_aug_ + 1; i++) {
    weights_(i) = 0.5/(n_aug_ + lambda_);
  }

}

UKF::~UKF() {}


/**
 * @param {Measurement} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(Measurement & meas_package) {
  double delta_t = (meas_package.timestamp_ - time_us_)/1E6;
  if(!is_initialized_){
    meas_package.get_ctrv_state(x_);
    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;
    return;
  }
  if(meas_package.sensor_type_ == Measurement::SensorType::LASER && ! use_laser_)
    return;
  if(meas_package.sensor_type_ == Measurement::SensorType::RADAR && ! use_radar_)
    return;

  Prediction(delta_t);
  time_us_ = meas_package.timestamp_;
  Update(meas_package);
  if(isnan(x_(0)) || fabs(x_(0)) > 1000 || fabs(x_(1)) > 1000|| fabs(x_(2)) > 1000|| fabs(x_(3)) > 1000|| fabs(x_(4)) > 1000 ) {
    cout << "bad value for x" << endl;
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
 
  //create augmented mean state
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented state covariance matrix
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  double spread = sqrt(lambda_+n_aug_);
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + spread * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - spread * L.col(i);
  }


  //predict sigma points
  Xsig_pred_ = MatrixXd(5, 15);
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = normalize_angle(yaw + yawd*delta_t);
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    normalize_ctrv(x_);
  }

  // cout << "Xsig_pred" << endl << "---------------" << endl << Xsig_pred_ << endl;
  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    normalize_ctrv(x_diff);

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }


}


/**
 * Updates the state and the state covariance matrix using a measurement.
 * @param {Measurement} meas_package
 */
void UKF::Update(Measurement & meas_package) {

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = meas_package.n_z;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points
    VectorXd z = VectorXd(meas_package.n_z);
    meas_package.state_to_measure(Xsig_pred_.col(i),z);
    Zsig.col(i) = z;
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    meas_package.normalize_measure(z_diff);

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  S = S + meas_package.R;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    meas_package.normalize_measure(z_diff);

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    normalize_ctrv(x_diff);

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  VectorXd z = meas_package.raw_measurements_;
  VectorXd z_diff = z - z_pred;
  meas_package.normalize_measure(z_diff);


  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  // calculate NIS
  NIS_ = z_diff.transpose() * S.inverse() * z_diff;
}
