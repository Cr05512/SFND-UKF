#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  is_initialized_ = false;

  n_x_ = 5;

  n_noise_ = 2;

  // initial state vector
  x_ = VectorXd(n_x_);
  x_.fill(0);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.2;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  n_z_lid_ = 2;
  n_z_rad_ = 3;

  Q_ = MatrixXd(n_noise_,n_noise_);
  Q_ << pow(std_a_,2), 0,
        0, pow(std_yawdd_,2);

  R_radar_ = MatrixXd(n_z_rad_,n_z_rad_);
  R_radar_ << pow(std_radr_,2), 0, 0,
              0, pow(std_radphi_,2), 0,
              0, 0, pow(std_radrd_,2);

  R_lidar_ = MatrixXd(n_z_lid_,n_z_lid_);
  R_lidar_ << pow(std_laspx_,2), 0,
              0, pow(std_laspy_,2); 

  n_aug_ = n_x_+n_noise_;

  x_aug_ = VectorXd(n_aug_);
  P_aug_ = MatrixXd(n_aug_,n_aug_);
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(n_x_,n_x_) = P_;
  P_aug_.bottomRightCorner(n_noise_,n_noise_) = Q_;
  Xsig_aug_ = MatrixXd(n_aug_,2*n_aug_+1);
  lambda_ = 3-n_aug_;
  Xsig_pred_ = MatrixXd(n_x_,2*n_aug_+1);
  x_pred_ = VectorXd(n_x_);

  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_/(lambda_+n_aug_);
  weights_.tail(2*n_aug_) = Eigen::MatrixXd::Ones(2*n_aug_,1)*0.5/(lambda_+n_aug_);

  time_us_ = 0.0;
  eps_ = 0.0;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package){
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if(!is_initialized_){

    if(use_laser_ && use_radar_){
      if(meas_package.sensor_type_==MeasurementPackage::LASER){
        x_[0] = meas_package.raw_measurements_[0];
        x_[1] = meas_package.raw_measurements_[1];
        std::cout << "Initialized by Lidar" << std::endl;
        is_initialized_ = true;
      }
    }
    else {
      if(use_laser_ && meas_package.sensor_type_==MeasurementPackage::LASER){
        x_[0] = meas_package.raw_measurements_[0];
        x_[1] = meas_package.raw_measurements_[1];
        std::cout << "Initialized by Lidar" << std::endl;
        is_initialized_ = true;
      }
      else if (use_radar_ && meas_package.sensor_type_==MeasurementPackage::RADAR){
        x_[0] = meas_package.raw_measurements_[0]*cos(meas_package.raw_measurements_[1]);
        x_[1] = meas_package.raw_measurements_[0]*sin(meas_package.raw_measurements_[1]);
        std::cout << "Initialized by Radar" << std::endl;
        is_initialized_ = true;
      }
    }
    
    P_ = 0.3*MatrixXd::Identity(5,5);
    
  }
  else{
    float dt = (meas_package.timestamp_-time_us_)/1000000.0;
    Prediction(dt);
    if(meas_package.sensor_type_==MeasurementPackage::LASER && use_laser_){
      UpdateLidar(meas_package);
    }
    else if(meas_package.sensor_type_==MeasurementPackage::RADAR && use_radar_){
      UpdateRadar(meas_package);
    }
  }
  time_us_ = meas_package.timestamp_;
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  //Augmentation
  x_aug_ << x_,
            VectorXd::Zero(n_noise_,1);
  
  P_aug_.topLeftCorner(n_x_,n_x_) = P_;

  MatrixXd A = P_aug_.llt().matrixL();

  // create augmented sigma points
  Xsig_aug_.col(0)  = x_aug_;
  for (int i = 0; i< n_aug_; ++i) {
    Xsig_aug_.col(i+1)       = x_aug_ + sqrt(lambda_+n_aug_) * A.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug_ - sqrt(lambda_+n_aug_) * A.col(i);
  }

  double p_x;
  double p_y;
  double v;
  double yaw;
  double yawd;
  double nu_a;
  double nu_yawdd;

  double px_p, py_p, v_p, yaw_p, yawd_p;

  // predict sigma points
  for(int i=0; i<2*n_aug_+1; i++){
      
    p_x = Xsig_aug_(0,i);
    p_y = Xsig_aug_(1,i);
    v = Xsig_aug_(2,i);
    yaw = Xsig_aug_(3,i);
    yawd = Xsig_aug_(4,i);
    nu_a = Xsig_aug_(5,i);
    nu_yawdd = Xsig_aug_(6,i);
      
    if(fabs(yawd)>0.0001){
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else{
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }
    
    v_p = v;
    yaw_p = yaw + yawd*delta_t;
    yawd_p = yawd;
    
    //we sum the noise contribution
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;
    
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
      
  }


  x_pred_.fill(0.0);
  for(int i=0; i<2*n_aug_+1; i++){
      x_pred_ = x_pred_ + weights_(i)*Xsig_pred_.col(i);
  }

  // predict state covariance matrix
  VectorXd psi = VectorXd(n_x_);
  P_.fill(0.0);
  for(int j=0; j<2*n_aug_+1;j++){
      psi = Xsig_pred_.col(j)-x_pred_;
      while (psi(3)> M_PI) psi(3)-=2.*M_PI;
      while (psi(3)<-M_PI) psi(3)+=2.*M_PI;
      P_ = P_ + weights_(j)*psi*psi.transpose();
  }

  x_ = x_pred_;
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  //std::cout << "Update by Lidar" << std::endl;
  
  MatrixXd Zsig = MatrixXd(n_z_lid_,2*n_aug_+1);
  Zsig_pred_ = MatrixXd(n_z_lid_,2*n_aug_+1);
  VectorXd z_pred = VectorXd(n_z_lid_);
  S_ = MatrixXd(n_z_lid_,n_z_lid_);

  // transform sigma points into measurement space
  for (int i = 0; i < 2*n_aug_+1; ++i) {
    // measurement model
    Zsig(0,i) = Xsig_pred_(0,i);
    Zsig(1,i) = Xsig_pred_(1,i);                         
  }

  z_pred.fill(0.0);

  for (int i=0; i < 2*n_aug_+1; ++i) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  VectorXd z_diff;
  VectorXd x_diff;
  MatrixXd Tc = MatrixXd(n_x_, n_z_lid_);
  Tc.fill(0.0);
  S_.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; ++i) {

    z_diff = Zsig.col(i) - z_pred;

    x_diff = Xsig_pred_.col(i) - x_;
     // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();

    S_ = S_ + weights_(i) * z_diff * z_diff.transpose();
  }

  S_ = S_ + R_lidar_;

  // Kalman gain K;
  MatrixXd K = Tc * S_.inverse();

  // residual
  z_diff = meas_package.raw_measurements_ - z_pred;

  eps_ = z_diff.transpose()*S_.inverse()*z_diff;
  NIS_lidar_.push_back(eps_);

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S_*K.transpose();
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  //std::cout << "Update by Radar" << std::endl;
  
  MatrixXd Zsig = MatrixXd(n_z_rad_,2*n_aug_+1);
  Zsig_pred_ = MatrixXd(n_z_rad_,2*n_aug_+1);
  VectorXd z_pred = VectorXd(n_z_rad_);
  S_ = MatrixXd(n_z_rad_,n_z_rad_);

  for (int i = 0; i < 2*n_aug_+1; ++i){
    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                       // r
    Zsig(1,i) = atan2(p_y,p_x);                                // phi
    Zsig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   // r_dot
  }

  // mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; ++i) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  VectorXd z_diff;
  // innovation covariance matrix S
  S_.fill(0.0);
  MatrixXd Tc = MatrixXd(n_x_, n_z_rad_);
  VectorXd x_diff;
  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; ++i){
    // residual
    z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

     // state difference
    x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    S_ = S_ + weights_(i) * z_diff * z_diff.transpose();
  }

  S_ = S_ + R_radar_;

  // Kalman gain K;
  MatrixXd K = Tc * S_.inverse();

  // residual
  z_diff = meas_package.raw_measurements_ - z_pred;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  eps_ = z_diff.transpose()*S_.inverse()*z_diff;
  NIS_radar_.push_back(eps_);

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S_*K.transpose();

}
