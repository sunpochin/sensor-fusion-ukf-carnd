#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  // P_ = MatrixXd(5, 5);
  P_ = MatrixXd::Zero(5, 5);


  // quote project tips :
  // "You will need to tune the process noise parameters std_a_ and std_yawdd_ in order to get your solution working on both datasets."
  // "The measurement noise parameters for lidar and radar should be left as given."
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;
  // todo: these two using 1, 1 looked fine with Radar data only. May need to be changed to meet rubric after adding lidar data.

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
}

UKF::~UKF() {}



/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  // copied from ekf project.
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/

  if (!is_initialized_) {
    // important! of   Prediction(delta_t); will fail due to too large values.
    time_us_ = meas_package.timestamp_ ;

    /// Initializing. codes from lecture: "What to Expect from the Project"
    // Initializing the State Vector x
    // try all 0.1 .
    x_ << 0.1, 0.1, 0.1, 0.1, 0.1;

    // Initializing the State Covariance Matrix P
    P_ <<
    1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0 ;

    // could experiment with "high uncertainty in px, py & v" if I like:
    // P_ <<
    // 1000, 0.0, 0.0, 0.0, 0.0,
    // 0.0, 1000, 0.0, 0.0, 0.0,
    // 0.0, 0.0, 1000, 0.0, 0.0,
    // 0.0, 0.0, 0.0, 7.0, 0.0,
    // 0.0, 0.0, 0.0, 0.0, 7.0 ;


    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates and initialize state.
      cout << "RADAR : measurement_pack.raw_measurements_ : " << meas_package.raw_measurements_ << endl;
      // double rho = meas_package.raw_measurements_(0);
      // double phi = meas_package.raw_measurements_(1);
      // double rho_dot = meas_package.raw_measurements_(2);
      // double px = cos(phi) * rho;
      // double py = sin(phi) * rho;
      // double vx = cos(phi) * rho_dot;
      // double vy = sin(phi) * rho_dot;
      // ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
      // cout << "x_ : " << x_ << endl;
      // x_ <<  px, py, rho, phi, rho_dot;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state.
      cout << "LASER measurement_pack :" << endl << meas_package.raw_measurements_ << endl;
      // double px = meas_package.raw_measurements_(0);
      // double py = meas_package.raw_measurements_(1);
      // double vx = 0.0;
      // double vy = 0.0;
      ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
      // x_ <<  px, py, vx, vy;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
  // end copied from ekf project.


  // copied from ekf project.
  // end copied from ekf project.
  // CRITERIA: Your Kalman Filter algorithm first predicts then updates.

  // cout << "timestamp_ :" << meas_package.timestamp_ << endl;
  long long delta_t = meas_package.timestamp_ - time_us_ ;
  time_us_ = meas_package.timestamp_ ;

  Prediction(delta_t);

  // copied from ekf project.
  // end copied from ekf project.
  // CRITERIA: Your Kalman Filter can handle radar and lidar measurements.
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // cout << "meas_package sensor_type_:  RADAR " << endl ; // << meas_package.sensor_type_ << endl;
    // UpdateRadar(meas_package) ;
  } else {
//    cout << "meas_package sensor_type_: Lidar " << endl ; // << meas_package.sensor_type_ << endl;
    UpdateLidar(meas_package) ;
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
  bool bdebug = false;
  // bdebug = true;

  // important: must change delta_t from us to seconds, or the angle normalization in step 4 would be too large to perform.
  delta_t = delta_t / 1000000;
  if (bdebug) {
    cout << "Prediction, delta_t : " << delta_t << endl;
  }

  /// step 1.1: generate sigma points. from: Generating Sigma Points Assignment 1
  /// step 1.2: sigma point augmentation. from "UKF Augmentation Assignment 1"
  /// step 2: Predict sigma points. codes from "Sigma Point Prediction Assignment 1"
  /// step 3: Predict Mean And Covariance . from "Predicted Mean and Covariance Assignment 1"


  /// step 1.1: generate sigma points. from: Generating Sigma Points Assignment 1
  //create sigma point matrix
  // todo: move these initilization to Constructor?
  //set state dimension
  n_x_ = 5;
  lambda_ = 3 - n_x_ ;

  //create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();
  Xsig.col(0)  = x_;

  //calculate sigma points ...
  //set sigma points as columns of matrix Xsig
  //set remaining sigma points
  for (int i = 0; i < n_x_; i++) {
    Xsig.col(i + 1)         = x_ + sqrt(lambda_ + n_x_ ) * A.col(i);
    Xsig.col(i + 1 + n_x_)  = x_ - sqrt(lambda_ + n_x_ ) * A.col(i);
  }


  /// step 1.2: sigma point augmentation, from "UKF Augmentation Assignment 1"
  //create augmented mean state
  //set augmented dimension
  // todo: move these initilization to Constructor?
  n_aug_ = 7;
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
//  std::cout << "x_aug = " << std::endl << x_aug << std::endl;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_ ;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;
//  std::cout << "P_aug = " << std::endl << P_aug << std::endl;
  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i+1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }


  /// step 2: Predict sigma points. codes from "Sigma Point Prediction Assignment 1"
  //create matrix with predicted sigma points as columns
  // Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  // Xsig_pred_.fill(0.0);
  Xsig_pred_ = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {

    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);

    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);

    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    //predicted state values
    double px_p, py_p;
    //avoid division by zero
    if ( fabs(yawd) > 0.001 ) {
        px_p = p_x + v/yawd * ( sin(yaw + yawd * delta_t ) - sin(yaw)  );
        py_p = p_y + v/yawd * ( -cos(yaw + yawd * delta_t ) + cos(yaw)  );
//        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    } else {
        px_p = p_x + v * delta_t * cos(yaw);
        py_p = p_y + v * delta_t * sin(yaw);
    }
    //add noise
    px_p = px_p + 0.5 * nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a*delta_t*delta_t * sin(yaw);

    //write predicted sigma points into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;

    double v_p = v;
    v_p = v_p + nu_a*delta_t;
    Xsig_pred_(2,i) = v_p;

    double yaw_p = yaw + yawd*delta_t;
    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    Xsig_pred_(3,i) = yaw_p;

    double yawd_p = yawd;
    yawd_p = yawd_p + nu_yawdd*delta_t;
    Xsig_pred_(4,i) = yawd_p;
  }
  if (bdebug) {
    std::cout << "Xsig_pred_ : " << std::endl << Xsig_pred_ << std::endl ;
  }

// return;

  /// step 3: Predict Mean And Covariance . from "Predicted Mean and Covariance Assignment 1"
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int it = 1; it < 2 * n_aug_ + 1; it++) {
      weights_(it) = 1 / (2 * (lambda_ + n_aug_) );
      // double weight = 0.5 / (lambda + n_aug);
      // weights(it) = weight;
  }
  // std::cout << "weights : " << weights_ << std::endl ;

  //predict state mean
  x_.fill(0.0);
  for (int it = 0; it < 2 * n_aug_ + 1; it++) {
      VectorXd tempmul = weights_(it) * Xsig_pred_.col(it) ;
      // std::cout << "weights : " << weights_(it) << std::endl ;
      // std::cout << "Xsig_pred.col(it)  : " << Xsig_pred_.col(it) << std::endl ;
      // std::cout << "tempmul  : " << tempmul << std::endl ;
      x_ = x_ + weights_(it) * Xsig_pred_.col(it) ;
  }
  std::cout << "x_ : " << std::endl << x_ << std::endl ;
  // predict state covariance matrix
  P_.fill(0.0);
  for (int it = 0; it < 2 * n_aug_ + 1; it++) {
      // state difference
      VectorXd x_diff = Xsig_pred_.col(it) - x_;
      //angle normalization
      while (x_diff(3)>M_PI) {
          x_diff(3)-=2.*M_PI;
          // std::cout << "x_diff(3) > : " << x_diff(3) << std::endl ;
          // std::cout << "M_PI : " << M_PI << std::endl ;
      }
      while (x_diff(3)<-M_PI) {
          x_diff(3)+=2.*M_PI;
          // std::cout << "x_diff(3) <- : " << x_diff(3) << std::endl ;
          // std::cout << "M_PI : " << M_PI << std::endl ;
      }
      P_ = P_ + weights_(it) * x_diff * x_diff.transpose() ;
  }

}


/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  // cout << "UpdateRadar : " << endl;

  bool bdebug = false;
  // bdebug = true;

  /// step 4: Predict Measurement. codes from Predict Radar Measurement Assignment.
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }
  if (bdebug) {
    std::cout << "Zsig : "  << std::endl << Zsig << std::endl;
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  if (bdebug) {
    std::cout << "z_pred : "  << std::endl << z_pred << std::endl;
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)>M_PI) {
      z_diff(1) -= 2.*M_PI;
      if (bdebug) {
        std::cout << "z_diff(1) > : " << z_diff(1) << std::endl ;
      }
    }
    while (z_diff(1)<-M_PI) {
      z_diff(1) += 2.*M_PI;
      if (bdebug) {
        std::cout << "z_diff(1) <- : " << z_diff(1) << std::endl ;
      }
    }
    S = S + weights_(i) * z_diff * z_diff.transpose();
    if (bdebug) {
      std::cout << "z_diff : "  << std::endl << z_diff << std::endl;
    }
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R <<    std_radr_ * std_radr_, 0, 0,
          0, std_radphi_ * std_radphi_, 0,
          0, 0, std_radrd_ * std_radrd_;
  S = S + R;
  if (bdebug) {
    std::cout << "S : "  << std::endl << S << std::endl;
  }

// return;
  /// step 5: Update state. UKF update.
  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_ ;

  if (bdebug) {
    std::cout << "calculate cross correlation matrix : "  << std::endl ;
  }
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  if (bdebug) {
    std::cout << "z_pred : "  << std::endl << z_pred << std::endl;
  }
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1) > M_PI) {
       z_diff(1)-=2.*M_PI;
       if (bdebug) {
         std::cout << "z_diff(1) > : " << z_diff(1) << std::endl ;
       }
    }
    while (z_diff(1)<-M_PI) {
      z_diff(1)+=2.*M_PI;
      if (bdebug) {
        std::cout << "z_diff(1) <- : " << z_diff(1) << std::endl ;
      }
    }

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    // std::cout << "iteration : " << i << " ";
    while (x_diff(3)>M_PI) {
      // std::cout << "before: in state difference, x_diff(3) > : " << x_diff(3) << " M_PI : " << M_PI << std::endl ;
      x_diff(3)-=2.*M_PI;
      // std::cout << "after: in state difference, x_diff(3) > : " << x_diff(3) << " M_PI : " << M_PI << std::endl ;
    }
    while (x_diff(3)<-M_PI) {
      // std::cout << "before: in state difference, x_diff(3) <- : " << x_diff(3) << " M_PI : " << M_PI << std::endl ;
      x_diff(3)+=2.*M_PI;
      // std::cout << "after: in state difference, x_diff(3) <- : " << x_diff(3) << " M_PI : " << M_PI << std::endl ;
    }
    // todo:
    // strange value with dataset 2:

    // iteration : 0 iteration : 1 iteration : 2
    // before: in state difference, x_diff(3) <- : -3.21725 M_PI : 3.14159
    // after: in state difference, x_diff(3) <- : 3.06594 M_PI : 3.14159
    // iteration : 3 iteration : 4 iteration : 5 iteration : 6 iteration : 7 iteration : 8 iteration : 9
    // before: in state difference, x_diff(3) > : 3.21725 M_PI : 3.14159
    // after: in state difference, x_diff(3) > : -3.06594 M_PI : 3.14159
    // iteration : 10 iteration : 11 iteration : 12 iteration : 13 iteration : 14
    // It turned out these 2 "caused by big std_a_1.png" and "caused by big std_a_2.png"
    // was caused by not initializing std_a_ and std_yawdd_ correctly.

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    // std::cout << "weights_(i) : " << std::endl << weights_(i) << std::endl ;
    // std::cout << "x_diff : " << std::endl << x_diff << std::endl ;
    if (bdebug) {
      std::cout << "z_diff : " << std::endl << z_diff << std::endl ;
      std::cout << "z_diff.transpose() : " << std::endl << z_diff.transpose() << std::endl ;
      std::cout << "Tc : " << Tc << std::endl ;
    }
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  if (bdebug) {
    std::cout << "Kalman gain K : " << K << std::endl ;
  }
  //residual
  VectorXd z_diff = z - z_pred;
  //angle normalization
  while (z_diff(1) > M_PI) {
    z_diff(1)-=2.*M_PI;
    // std::cout << "z_diff(1) > : " << z_diff(1) << std::endl ;
  }
  while (z_diff(1)<-M_PI) {
    z_diff(1)+=2.*M_PI;
    // std::cout << "z_diff(1) <- : " << z_diff(1) << std::endl ;
  }
  //update state mean and covariance matrix
  if (bdebug) {
    std::cout << "update state mean and covariance matrix : " << std::endl ;
  }
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

}




/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  cout << "UpdateLidar : measurement_pack.raw_measurements_ : " << meas_package.raw_measurements_ << endl;

  //measurement covariance matrix - laser
  MatrixXd R_laser_ = MatrixXd::Zero(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;
  MatrixXd R_ = R_laser_;

  // MatrixXd H_laser_ = MatrixXd::Zero(2, 4);
  // H_laser_ << 1, 0, 0, 0,
  //             0, 1, 0, 0 ;
  MatrixXd H_laser_ = MatrixXd::Zero(2, 5);
  H_laser_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0;
  MatrixXd H_ = H_laser_;
  VectorXd z_pred = H_ * x_;

  VectorXd z = VectorXd::Zero(2);
  z << meas_package.raw_measurements_ ;
  VectorXd y = z - z_pred;
  // cout << " z : " << endl << z << endl;
  // cout << " z_pred : " << endl << z_pred << endl ;
  // cout << " y : " << endl << y << endl ;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}
