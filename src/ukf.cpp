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
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.52; //30deg/s^2
  
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
  
  is_initialized_ = false;

  previous_timestamp_= 0;

  n_x_ = x_.size();

  n_aug_ = n_x_ + 2; // augmenting with longitudinal acceleration and yaw accelaration

  Xsig_pred_ = MatrixXd(n_x_,2 * n_aug_+ 1 );

  weights_ = VectorXd(2*n_aug_ + 1);

  lambda_ = 3 - n_aug_;

  R_laser_ = MatrixXd(2,2);
  R_laser_ << std_laspx_ * std_laspx_, 0,
                       0,  std_laspy_ * std_laspy_;

  R_radar_ = MatrixXd(3,3);
  R_radar_ << std_radr_ * std_radr_, 0, 0,
                  0, std_radphi_*std_radphi_, 0,
                  0,  0, std_radrd_ * std_radrd_;

  NIS_laser_ = 0.0;
  NIS_radar_ = 0.0;
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
    if(!is_initialized_) {
        // initialize state x_,  covariance P_ and previous_timestamp_
        InitializeState(meas_package);
        is_initialized_ = true;
        return;
    }

    double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0; // dt in secs
    cout << "DEBUG: Calling PRediction UKF\n";
    // Kalman Filter - Prediction - Update steps

    // call Prediction function to predict the state for the interval dt
    Prediction(dt);

    // update based on the current sensor type
    if(meas_package.sensor_type_ == MeasurementPackage::LASER) {
        cout << "DEBUG: Calling Update Lidar UKF\n";
        UpdateLidar(meas_package);
    } else if(meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        cout << "DEBUG: Calling Update RADAR \n";
        UpdateRadar(meas_package);
    }
    cout << "DEBUG: Update UKF DONE\n";
    // record the current timestamp in previous_timestamp_ variable for the next iteration
    previous_timestamp_ = meas_package.timestamp_;

}

void UKF::InitializeState(MeasurementPackage meas_package) {
    cout << "DEBUG: Init UKF\n";
    double px, py,v=0.0, yaw=0.0, yaw_rate=0.0;
    cout << "DEBUG: About to init sensor type " << meas_package.sensor_type_ <<"\n";
    if(meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        double rho = meas_package.raw_measurements_[0];
        double phi = meas_package.raw_measurements_[1];
        // we don't need rho dot as we are not interested in the  radar velocity (velocity from the radar to the object)
        // in the CTRV model. We initialize v to 0 in the CTRV model
        //double rho_dot = meas_package.raw_measurements_[2];

        px = rho * cos(phi);
        py = rho * sin(phi);

    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        px = meas_package.raw_measurements_[0];
        py = meas_package.raw_measurements_[1];
    } else {
        throw std::invalid_argument("Invalid sensor type when trying to initialize state");
    }

    cout << "DEBUG: After Initializing px and py\n";
    // initialize state vector
    x_ << px, py, v, yaw, yaw_rate;
    // initialize covariance matrix
    P_ << 0.02,0,0,0,0,
          0,0.02,0,0,0,
          0,0,1.0,0,0,
          0,0,0,0.1,0,
          0,0,0,0,0.1;
    // set previous_timestamp_
    previous_timestamp_ = meas_package.timestamp_;

    cout << "DEBUG: Initializing weights_\n";
    // set weights
    weights_(0) = lambda_/(lambda_ + n_aug_) ;
    for(int i =1; i<2*n_aug_ + 1; i++)
        weights_(i) = 0.5/(lambda_ + n_aug_);

    cout << "DEBUG: End Initializing UKF \n";

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
    cout << "DEBUG: In PRediction UKF\n";
    // generate augmented sigma points
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_ +1);

    //create augmented state vector
    VectorXd x_aug = VectorXd(n_aug_);
    x_aug.head(n_x_) = x_;
    x_aug(n_x_) = 0.0;
    x_aug(n_x_+1) = 0.0;

    // create augmented covariance matrix
    MatrixXd P_aug = MatrixXd(n_aug_,n_aug_);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_,n_x_) = P_;
    P_aug(n_x_,n_x_) = std_a_* std_a_; // linear acceleration cov
    P_aug(n_x_+1, n_x_+1) = std_yawdd_ * std_yawdd_; // yaw accelartion cov

    cout << "DEBUG: Prediction UKF :: Creating sqrt matrix L\n";
    MatrixXd L = P_aug.llt().matrixL(); // sqrt matrix

    Xsig_aug.col(0) = x_aug;
    for(int i =0; i<n_aug_; i++) {
        Xsig_aug.col(i+1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
    }
    cout << "DEBUG: Prediction UKF :: Created Sigma Points \n";
    // now run the process function through the sigma points
    for(int i =0; i< 2*n_aug_+1; i++) {
        double px = Xsig_aug(0,i);
        double py = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yaw_rate = Xsig_aug(4,i);
        double nu_acc = Xsig_aug(5,i);
        double nu_yaw_acc = Xsig_aug(6,i);

        // predict the x state values from teh above sigma points by running them through process function
        double px_p, py_p =0;
        // handle divide by zero
        if(fabs(yaw_rate) > 0.001) {
            px_p = px + v/yaw_rate * ( sin(yaw + yaw_rate * delta_t) - sin(yaw));
            py_p = py + v/yaw_rate * ( cos(yaw) - cos(yaw + yaw_rate * delta_t));
        } else {
            px_p = px + v * delta_t * cos(yaw);
            py_p = py + v * delta_t * sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yaw_rate * delta_t;
        double yaw_rate_p = yaw_rate;

        // add noise
        px_p = px_p + 1/2.0 * nu_acc * delta_t * delta_t * cos(yaw);
        py_p = py_p + 1/2.0 * nu_acc * delta_t * delta_t * sin(yaw);
        v_p = v_p + nu_acc * delta_t;
        yaw_p = yaw_p + 1/2.0 * nu_yaw_acc * delta_t * delta_t;
        yaw_rate_p = yaw_rate_p + nu_yaw_acc * delta_t;

        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yaw_rate_p;

    }
    cout << "DEBUG: Prediction UKF :: Created Predictions on Sigma Points \n";

    // From the Predicted sigma points compute the predict state and covariance
    // mean prediction
    x_.fill(0.0);
    for(int i =0; i< 2* n_aug_ + 1; i++) {
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }
    cout << "DEBUG: Prediction UKF :: Estimated x_ from sigma points \n";

    // covariance prediction
    P_.fill(0.0);
    for(int i =0; i < 2* n_aug_ + 1; i++) {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // normalize yaw angle because difference could include 2PI multiple
        while(x_diff(3) > M_PI ) x_diff(3) -= 2*M_PI;
        while(x_diff(3) < -M_PI) x_diff(3) += 2*M_PI;
        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
    }
    cout << "DEBUG: Prediction UKF :: Estimated P_ from sigma points \n";

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
    VectorXd z = meas_package.raw_measurements_;
    int n_z = 2; // laser has only 2 measurement variables - px and py

    MatrixXd zSig = MatrixXd(n_z, 2 * n_aug_ + 1);

    // reusing sigma points from Xsig_pred_ run it through function h(x) to predict the sigma
    // points in the measurement space. For laser h(x) = x
    for(int i =0; i< 2* n_aug_ + 1; i++) {
        zSig(0,i) = Xsig_pred_(0,i); //px
        zSig(1,i) = Xsig_pred_(1,i); // py
    }

    //now do the mean measurement predictions
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for(int i =0; i< 2 * n_aug_ + 1; i++) {
        z_pred = z_pred + weights_(i) * zSig.col(i);
    }
    //measurement covariance pred - S
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);
    for(int i=0; i < 2 * n_aug_ + 1; i++) {
        VectorXd zSig_diff_w_mean = zSig.col(i) - z_pred;

        S = S + weights_(i) * zSig_diff_w_mean * zSig_diff_w_mean.transpose();
    }
    // measurement noise cov - additive
    S = S + R_laser_;  // this is the part used in calculating gain g=p/(p+r) = p *(p+r)^-1 ,
                                                  // where S= p+r = H*P*Ht + R
                                                  //      H*P*Ht - translates to measurement space

    // Compute the cross corelation matrix Tc -- cross corelation between measurement and state covariance
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);
    for(int i =0; i< 2 * n_aug_ + 1; i++ ) {

        VectorXd z_diff = zSig.col(i) - z_pred;
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // For performance - do the inverse only once as we use this in K and NIS computes
    MatrixXd S_inv = S.inverse();

    // Kalman Gain
    MatrixXd K = Tc * S_inv;

    // y residual
    VectorXd y = z - z_pred;

    // consistency check - NIS
    NIS_laser_ = y.transpose() * S_inv * y;

    // update state and covariance
    x_ = x_ + K * y;
    P_ = P_ - K * S * K.transpose();

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
    VectorXd z = meas_package.raw_measurements_;
    int n_z = 3;

    MatrixXd zSig = MatrixXd(n_z, 2 * n_aug_ + 1);

    // reusing sigma points from Xsig_pred_ run it through function h(x) to predict the sigma
    // points in the measurement space.
    for(int i =0; i< 2* n_aug_ + 1; i++) {

        double px = Xsig_pred_(0,i);
        double py = Xsig_pred_(1,i);
        double v = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);

        double vx = v * cos(yaw);
        double vy = v * sin(yaw);

        zSig(0,i) =  sqrt(px * px + py * py); //rho
        zSig(1,i) =  atan2(py, px);         // phi
        zSig(2,i) =  (px * vx + py * vy)/zSig(0,i); // rho_dot
    }

    //  mean measurement prediction
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for(int i =0; i< 2 * n_aug_ + 1; i++)  {
        z_pred = z_pred + weights_(i) * zSig.col(i);
    }

    // measurement covariance
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);
    for(int i = 0; i < 2 * n_aug_ + 1; i++ ) {
        VectorXd z_diff_from_mean = zSig.col(i) - z_pred;

        while(z_diff_from_mean(1) >  M_PI) z_diff_from_mean(1) -= 2 * M_PI;
        while(z_diff_from_mean(1) < -M_PI) z_diff_from_mean(1) += 2 * M_PI;

        S = S + weights_(i) * z_diff_from_mean * z_diff_from_mean.transpose();
    }
    // add radar noise covariance matrix (additive)
    S = S + R_radar_;

    // Compute cross relational matrix Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);
    for(int i =0; i< 2 * n_aug_ + 1; i ++) {

        VectorXd z_diff = zSig.col(i) - z_pred;
        // phi angle normalization
        while(z_diff(1) >  M_PI) z_diff(1) -= 2 * M_PI;
        while(z_diff(1) < -M_PI) z_diff(1) += 2 * M_PI;

        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // yaw angle normalization
        while(x_diff(3) >  M_PI) x_diff(3) -= 2 * M_PI;
        while(x_diff(3) < -M_PI) x_diff(3) += 2 * M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // For performance - do the inverse only once as we use this in K and NIS computes
    MatrixXd S_inv = S.inverse();

    // Kalman Gain
    MatrixXd K = Tc * S_inv;

    // y residual
    VectorXd y = z - z_pred;

    // phi angle normalization in y residual
    while(y(1) >  M_PI) y(1) -= 2 * M_PI;
    while(y(1) < -M_PI) y(1) += 2 * M_PI;

    // update state and covariance
    x_ = x_ + K * y;
    P_ = P_ - K * S * K.transpose();

    // consistency check - NIS
    NIS_radar_ = y.transpose() * S_inv * y;
}
