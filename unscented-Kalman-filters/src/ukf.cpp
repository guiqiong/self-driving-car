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
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.25;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.2;

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
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
    is_initialized_ = false;
    
    time_us_ = 0;
    
    
    n_x_ = 5;
    n_aug_ = 7;
    lambda_ = 3 - n_aug_;
    
    
    Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);
    
    
    weights_ = VectorXd(2*n_aug_+1);
    double weight_0 = lambda_/(lambda_+n_aug_);
    weights_(0) = weight_0;
    for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
        double weight = 0.5/(n_aug_+lambda_);
        weights_(i) = weight;
    }
    
    //long long previous_timestamp_ = 0;
    
    //x_ << 1,1,1,1,1;

    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;
    
    th = 0.001;
    
    R_laser_ = MatrixXd(2,2);
    R_laser_ << std_laspx_*std_laspx_, 0,
                0,                     std_laspy_*std_laspy_;
    
    
    R_radar_ = MatrixXd(3,3);
    R_radar_ << std_radr_*std_radr_, 0,                     0,
                0,                 std_radphi_*std_radphi_, 0,
                0,                 0,                     std_radrd_*std_radrd_;
    

    
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
    if (!is_initialized_) {
        /*
         * Initialize the state x_ with the first measurement.
         * Create the covariance matrix.
         * Convert radar from polar to cartesian coordinates.
         */
        // first measurement
        cout << "UKF: " << endl;
        double px, py, vx, vy;
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            /**
             Convert radar from polar to cartesian coordinates and initialize state.
             */
            double rho = meas_package.raw_measurements_[0];
            double phi = meas_package.raw_measurements_[1];
            double drho = meas_package.raw_measurements_[2];
            
            px = rho*cos(phi);
            py = rho*sin(phi);
            vx = drho*cos(phi);
            vy = drho*sin(phi);
        }
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            /**
             Initialize state.
             */
            px = meas_package.raw_measurements_[0];
            py = meas_package.raw_measurements_[1];
            vx = 0;
            vy = 0;
        }
        
        if(fabs(px)<th){
            px = th;
        }
        if(fabs(py)<th){
            py = th;
        }
        x_ << px, py, sqrt(vx*vx+vy*vy), 0, 0;
        
        
        time_us_ = meas_package.timestamp_;
        
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }
    
    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    
    double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
    time_us_ = meas_package.timestamp_;
    
    Prediction(dt);
    
    /*****************************************************************************
     *  Update
     ****************************************************************************/
    
    /**
     * Use the sensor type to perform the update the state and covariance matrices.
     */
    
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        UpdateRadar(meas_package);
    } else {
        // Laser updates
        //measurement update
        UpdateLidar(meas_package);
    }
}


void UKF::Augmentation(){
    
    //create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);
    
    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    
    
    //create augmented mean state
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;
    
    //create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5,5) = P_;
    P_aug(5,5) = std_a_*std_a_;
    P_aug(6,6) = std_yawdd_*std_yawdd_;
    
    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();
    
    //create augmented sigma points
    Xsig_aug_ = MatrixXd(n_aug_, 2*n_aug_+1);
    Xsig_aug_.col(0)  = x_aug;
    for (int i = 0; i< n_aug_; i++)
    {
        Xsig_aug_.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
        Xsig_aug_.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
    }

}

void UKF::PredictSigmaPoints(double delta_t){
    
    //predict sigma points
    for (int i = 0; i< 2*n_aug_+1; i++)
    {
        //extract values for better readability
        double p_x = Xsig_aug_(0,i);
        double p_y = Xsig_aug_(1,i);
        double v = Xsig_aug_(2,i);
        double yaw = Xsig_aug_(3,i);
        double yawd = Xsig_aug_(4,i);
        double nu_a = Xsig_aug_(5,i);
        double nu_yawdd = Xsig_aug_(6,i);
        
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
        double yaw_p = yaw + yawd*delta_t;
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
}

void UKF::PredictMeanAndCovariance(){
    
    //predicted state mean
    VectorXd x = VectorXd(n_x_);
    x.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        x = x + weights_(i) * Xsig_pred_.col(i);
    }
    
    //predicted state covariance matrix
    MatrixXd P = MatrixXd(n_x_, n_x_);
    P.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - Xsig_pred_.col(0);
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        
        P = P + weights_(i) * x_diff * x_diff.transpose() ;
    }
    
    x_ = x;
    P_ = P;

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
    
    Augmentation();
    PredictSigmaPoints(delta_t);
    PredictMeanAndCovariance();

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
    int n_z = 2;
    MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
    
    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        
        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        
        // measurement model
        if(fabs(p_x)>0.001 || fabs(p_y)>0.001){
            Zsig(0,i) = p_x;                        //px
            Zsig(1,i) = p_y;                        //py
        }else{
            Zsig(0,i) = 0.001;                        //px
            Zsig(1,i) = 0.001;
        }
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
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        
        S = S + weights_(i) * z_diff * z_diff.transpose();
    }
    
    //add measurement noise covariance matrix
    S = S + R_laser_;
    
    
    //create example vector for incoming radar measurement
    VectorXd z = VectorXd(n_z);
    z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];
    
    
    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        
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
    
    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();
    
    //residual
    VectorXd z_diff = z - z_pred;
    
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();
    
    
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
    int n_z = 3;
    MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
    
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
        if(fabs(p_x)>th || fabs(p_y)>th){
            Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
            Zsig(1,i) = atan2(p_y,p_x);                                 //phi
            Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
        }else{
            Zsig(0,i) = 0;                        //r
            Zsig(1,i) = 0;                                 //phi
            Zsig(2,i) = 0;
        }
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
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        
        S = S + weights_(i) * z_diff * z_diff.transpose();
    }
    
    //add measurement noise covariance matrix
    S = S + R_radar_;
    
    
    //create example vector for incoming radar measurement
    VectorXd z = VectorXd(n_z);
    z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];
    
    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        
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
    
    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();
    
    //residual
    VectorXd z_diff = z - z_pred;
    
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();
    
    
    
}


