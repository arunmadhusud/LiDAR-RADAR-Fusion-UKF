#include "ukf.h"
#include "Eigen/Dense"

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

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_.setIdentity(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2;
  
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
  // set state dimension
  n_x_ = 5;

  // set augmented dimension
  n_aug_ = 7;

  // set is_initialized_ to false
  is_initialized_ = false;

  // find lambda
  lambda_ = 3 - n_aug_;

  // set weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i=1; i < 2 * n_aug_ + 1; i++ )
  {
    weights_(i) = 0.5/(lambda_+n_aug_);
  }

   
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (!is_initialized_)
  {
    if (meas_package.sensor_type_ == meas_package.LASER && use_laser_== true)
    {
      x_ << meas_package.raw_measurements_(0),
            meas_package.raw_measurements_(1),
            0,
            0,
            0;      
      P_(0,0) = std_laspx_ * std_laspx_;
      P_(1,1) = std_laspx_ * std_laspx_;
      long long time_us_ = meas_package.timestamp_;
      is_initialized_ = true;
      return;      
    }

    if (meas_package.sensor_type_ == meas_package.RADAR && use_radar_ == true)
    {
        double rho = meas_package.raw_measurements_(0);      // range
        double phi = meas_package.raw_measurements_(1);      // bearing
        double rho_dot = meas_package.raw_measurements_(2);  // radial velocity

        // Convert radar measurements from polar to Cartesian coordinates
        double px = rho * cos(phi);  // x position in ego vehicle's coordinate system
        double py = rho * sin(phi);  // y position in ego vehicle's coordinate system

        // Initialize state vector
        double v = rho_dot; // This is an estimate, may not be accurate for tangential velocity
        x_ << px, py, v, 0, 0; // yaw and yaw rate initialized to zero

        // Initialize the covariance matrix P_
        P_(0, 0) = std_radr_ * std_radr_;
        P_(1, 1) = std_radr_ * std_radr_;

        long long time_us_ = meas_package.timestamp_;
        is_initialized_ = true;
        return;
    }
  }

  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  Prediction(dt);

  if (meas_package.sensor_type_ == meas_package.LASER && use_laser_== true)
  {
    UpdateLidar(meas_package);
  }

  if (meas_package.sensor_type_ == meas_package.RADAR && use_radar_== true)
  {
    UpdateRadar(meas_package);
  }


  
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  // create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.fill(0.0);

  //created augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_,n_aug_);
  P_aug.fill(0.0);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.head(n_x_) = x_;

  //create augmented covraince matrix
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  MatrixXd Q = MatrixXd(2,2);
  Q << std_a_ * std_a_, 0, 0, std_yawdd_ * std_yawdd_;
  P_aug.bottomRightCorner(n_aug_-n_x_,n_aug_-n_x_) = Q;

  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i=0; i < n_aug_; i++){
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_) * A.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * A.col(i);
  }

  // create matrix with predicted sigma points as columns
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // predict sigma points

  for (int i=0; i < 2 * n_aug_ +1; i++)
  {
    if (fabs(Xsig_aug(4,i)) > 0.001)
    {
      Xsig_pred_(0,i) = Xsig_aug(0,i) + Xsig_aug(2,i)/Xsig_aug(4,i) * (sin(Xsig_aug(3,i) + Xsig_aug(4,i)*delta_t) - sin(Xsig_aug(3,i)));
      Xsig_pred_(1,i) = Xsig_aug(1,i) + Xsig_aug(2,i)/Xsig_aug(4,i) * (-cos(Xsig_aug(3,i) + Xsig_aug(4,i)*delta_t) + cos(Xsig_aug(3,i)));
      Xsig_pred_(2,i) = Xsig_aug(2,i);
      Xsig_pred_(3,i) = Xsig_aug(3,i) + Xsig_aug(4,i)*delta_t;
      Xsig_pred_(4,i) = Xsig_aug(4,i);
    }
    else
    {
      Xsig_pred_(0,i) = Xsig_aug(0,i) + Xsig_aug(2,i)*cos(Xsig_aug(3,i))*delta_t;
      Xsig_pred_(1,i) = Xsig_aug(1,i) + Xsig_aug(2,i)*sin(Xsig_aug(3,i))*delta_t;
      Xsig_pred_(2,i) = Xsig_aug(2,i);
      Xsig_pred_(3,i) = Xsig_aug(3,i) + Xsig_aug(4,i)*delta_t;
      Xsig_pred_(4,i) = Xsig_aug(4,i);
    }
    // add noise
    Xsig_pred_(0,i) += 0.5*delta_t*delta_t*cos(Xsig_aug(3,i))*Xsig_aug(5,i);
    Xsig_pred_(1,i) += 0.5*delta_t*delta_t*sin(Xsig_aug(3,i))*Xsig_aug(5,i);
    Xsig_pred_(2,i) += delta_t*Xsig_aug(5,i);
    Xsig_pred_(3,i) += 0.5*delta_t*delta_t*Xsig_aug(6,i);
    Xsig_pred_(4,i) += delta_t*Xsig_aug(6,i);
  }

  // predict state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // predict state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2* n_aug_ + 1; i++)
  { 
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  } 


}


void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  
  int n_z_ =3; // number of radar measurements
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_,2 * n_aug_ + 1);
  
  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_,n_z_);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
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

  // calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < Zsig.cols(); i++) {
    z_pred += weights_(i) * Zsig.col(i);
  }

  // calculate innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < Zsig.cols(); i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    
    // angle normalization for the phi component
    while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;
    
    S += weights_(i) * z_diff * z_diff.transpose();
  }

  // Define measurement noise covariance matrix R
  MatrixXd R = MatrixXd(n_z_,n_z_);
  R <<    std_radr_ * std_radr_, 0, 0,
          0, std_radphi_ * std_radphi_, 0,
          0, 0, std_radrd_ * std_radrd_;
  
  S = S + R;

  // create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z_);
  z << meas_package.raw_measurements_(0),
       meas_package.raw_measurements_(1),
       meas_package.raw_measurements_(2);
  
  // create matrix for cross correleation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);
  Tc.fill(0.0);
  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  // calculate NIS  
  NIS_radar_.push_back(z_diff.transpose() * S.inverse() * z_diff);

}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  int n_z_ = 2; // number of LiDAR measurements
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_,2 * n_aug_ + 1);
  
  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_,n_z_);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = p_x;                      // px
    Zsig(1,i) = p_y;                      // py
  }

  // calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < Zsig.cols(); i++) {
    z_pred += weights_(i) * Zsig.col(i);
  }

  // calculate innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < Zsig.cols(); i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
     
    S += weights_(i) * z_diff * z_diff.transpose();
  }

  // Define measurement noise covariance matrix R
  MatrixXd R = MatrixXd(n_z_,n_z_);
  R <<    std_laspx_ * std_laspx_, 0,
          0, std_laspy_ * std_laspy_;
  
  S = S + R;

  // create example vector for incoming LiDar measurement
  VectorXd z = VectorXd(n_z_);
  z << meas_package.raw_measurements_(0),
       meas_package.raw_measurements_(1);
  
  // create matrix for cross correleation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);
  Tc.fill(0.0);
  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  // calculate NIS
  NIS_laser_.push_back(z_diff.transpose() * S.inverse() * z_diff);
}