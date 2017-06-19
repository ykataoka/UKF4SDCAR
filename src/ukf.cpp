#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  cout << "Constructor..." << endl;
  
  /// initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;
  
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 5; // NEEDS to be manually adjusted - 1 := 3.6[km/h]/s

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.4; // NEEDS to be manually adjusted - 1 :=57.32[deg/s]/s

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // weights for mu and covariance using sigma points
  weights_ = VectorXd(2*n_aug_ + 1);  

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;
  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    
    // first measurement
    x_ = VectorXd(5);
    x_ << 0, 0, 0, 0, 0; // assuming we do not have simulation input

    // if RADAR
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Initialize state.
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho = meas_package.raw_measurements_(0);
      float phi = meas_package.raw_measurements_(1);
      float dphi = meas_package.raw_measurements_(2);
      x_(0) = rho * sin(phi);
      x_(1) = rho * cos(phi);
      x_(3) = phi;
      x_(4) = dphi;
    }

    // LASER
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {      
      /**
      Initialize state.
      */
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
    }

    // state covariance matrix
    P_ = MatrixXd(5, 5);
    // P_ << std_laspx_*std_laspx_, 0, 0, 0, 0,
    //   0, std_laspy_*std_laspy_, 0, 0, 0,
    //   0, 0, std_radr_*std_radr_, 0, 0,
    //   0, 0, 0, std_radphi_*std_radphi_, 0,
    //   0, 0, 0, 0, std_radrd_*std_radrd_;
    double dt = 0.1;
    double stdx = std_laspx_;
    double stdy = std_laspy_;
    double stdv = dt*std_a_;
    double stdpsi = 0.5*dt*dt*std_yawdd_;
    double stdpsid = dt*std_yawdd_;

    P_ << stdx*stdx, 0, 0, 0, 0,
      0, stdy*stdy, 0, 0, 0,
      0, 0, stdv*stdv, 0, 0,
      0, 0, 0, stdpsi*stdpsi, 0,
      0, 0, 0, 0, stdpsid*stdpsid;

    cout << "x_ = " << x_ << endl;
    cout << "P_ = " << P_ << endl;
    
    // done initializing, no need to predict or update
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;

    return;
  }
  
  
  /*****************************************************************************
   *  Prediction (State Estimation using Vehicle Dynamics)
   ****************************************************************************/
  // update the state transition matrix F according to the new elapsed time.
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;
  
  // predicts sigma points, the state, and the state covariance matrix.
  Prediction(dt);
  
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  // CASE : RADAR (Extended kalman Filter)
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
    
  // CASE : LASER (Standsard kalman Filter)
  }else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);  
  }

  // print the output
  //  cout << "x_ = " << x_ << endl;
  //  cout << "P_ = " << P_ << endl;
    
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
  
  /*--- State Augmentation at t = k---*/
  //create augmented mean vector
  VectorXd x_aug_ = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug_ = MatrixXd(7, 7);

  //create sigma point matrix
  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug_.head(5) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;

  //create augmented covariance matrix
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(5,5) = P_;
  P_aug_(5,5) = std_a_*std_a_;
  P_aug_(6,6) = std_yawdd_*std_yawdd_;


  /*--- Sigma Point at t = k ---*/
  MatrixXd L = P_aug_.llt().matrixL();   // square root matrix
  Xsig_aug_.col(0)  = x_aug_;
  for (int i = 0; i< n_aug_; i++)
    {
      Xsig_aug_.col(i+1)       = x_aug_ + sqrt(lambda_ + n_aug_) * L.col(i);
      Xsig_aug_.col(i+1+n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * L.col(i);
    }  
  
  /*--- Predict Sigma Point at t = k+1 ---*/
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  for (int i=0; i< 2*n_aug_+1; i++)
  {
    //extract values at t = k
    // for better readability
    double p_x_ = Xsig_aug_(0,i);
    double p_y_ = Xsig_aug_(1,i);
    double v_ = Xsig_aug_(2,i);
    double yaw_ = Xsig_aug_(3,i);
    double yawd_ = Xsig_aug_(4,i);
    double nu_a_ = Xsig_aug_(5,i);
    double nu_yawdd_ = Xsig_aug_(6,i);

    //predicted state values
    double px_p_, py_p_;
    double v_p_ = v_;
    double yaw_p_ = yaw_ + yawd_*delta_t;
    double yawd_p_ = yawd_;

    // process model step 1 : x_k+1 = f(x_k, nu_k) (without noise)
    if (fabs(yawd_) > 0.001) {  //avoid division by zero 
        px_p_ = p_x_ + v_/yawd_ * ( sin(yaw_ + yawd_ * delta_t) - sin(yaw_) );
        py_p_ = p_y_ + v_/yawd_ * ( cos(yaw_) - cos(yaw_ + yawd_ * delta_t) );
    }
    else {
        px_p_ = p_x_ + v_ * delta_t * cos(yaw_);
        py_p_ = p_y_ + v_ * delta_t * sin(yaw_);
    }

    // process model step 2 : add noise
    px_p_ = px_p_ + 0.5 * nu_a_ * delta_t * delta_t * cos(yaw_);
    py_p_ = py_p_ + 0.5 * nu_a_ * delta_t * delta_t * sin(yaw_);
    v_p_ = v_p_ + nu_a_ * delta_t;

    yaw_p_ = yaw_p_ + 0.5 * nu_yawdd_ * delta_t * delta_t;
    yawd_p_ = yawd_p_ + nu_yawdd_ * delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p_;
    Xsig_pred_(1,i) = py_p_;
    Xsig_pred_(2,i) = v_p_;
    Xsig_pred_(3,i) = yaw_p_;
    Xsig_pred_(4,i) = yawd_p_;
  }

  /*--- Predict new state x(mean and covariance) at t = k+1 ---*/

  // create vector for weights_
  double weight_0 = lambda_/(lambda_ + n_aug_); // case 0 : sigma point for mean
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_ + lambda_);
    weights_(i) = weight;
  }

  // predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_+ weights_(i) * Xsig_pred_.col(i); // vector(7x1) + scholar * vector(7x1)
  }
  
  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points    
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }  
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
     Use lidar data to update the belief about the object's
     position. Modify the state vector, x_, and covariance, P_.
  */
  int n_z = 2; // Lidar has 2 dimension
  VectorXd z = VectorXd(n_z);
  z = meas_package.raw_measurements_;  

  /*--- transform sigma points into measurement space z^(k+1|k) ---*/

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);

    // measurement model
    Zsig(0,i) = p_x;  //x
    Zsig(1,i) = p_y;  //y
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
  for (int i = 0; i < 2*n_aug_+1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R << std_laspx_*std_laspx_, 0,
    0, std_laspy_*std_laspy_;
  S = S + R;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);

  //calculate cross correlation matrix
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // Residual
  VectorXd z_diff = z - z_pred;

  // Angle normalization
  while (z_diff(1) >  M_PI) z_diff(1) -= 2.*M_PI;
  while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose(); 

  // NIS (Normalized Innovation Squared)
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
  //  cout << "NIS_laser_ (<95% : 5.991): " << NIS_laser_ << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
     Use radar data to update the belief about the object's
     position. Modify the state vector, x_, and covariance, P_.     
  */

  int n_z = 3; // Lidar has 3 dimension
  VectorXd z = VectorXd(n_z);
  z = meas_package.raw_measurements_;

  /*--- transform sigma points into measurement space z^(k+1|k) ---*/

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    
    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    
    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;
    
    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                      //r
    Zsig(1,i) = atan2(p_y,p_x);                               //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y); //r_dot
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
  for (int i = 0; i < 2*n_aug_+1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R << std_radr_*std_radr_, 0, 0,
       0, std_radphi_*std_radphi_, 0,
       0, 0,std_radrd_*std_radrd_;
  S = S + R;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);

  //calculate cross correlation matrix
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // Residual
  VectorXd z_diff = z - z_pred;

  // Angle normalization
  while (z_diff(1) >  M_PI) z_diff(1) -= 2.*M_PI;
  while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  // NIS (Normalized Innovation Squared)
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  //  cout << "NIS_radar_ (<95% : 7.815): " << NIS_radar_ << endl;
  
}
