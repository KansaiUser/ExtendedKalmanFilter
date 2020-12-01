#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * Finish initializing the FusionEKF.
   * Set the process and measurement noises
   * That mean process (Q) and Measurement R
   */

  // create a 4D state vector, we don't know yet the values of the x state
  //ekf_.x_ = VectorXd(4);   //Not necessary

  // state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

  
  H_laser_<< 1.,0.,0.,0.,
              0.,1.,0.,0.;


  //ekf_.Initialize(H_laser_ ,R_laser_); 

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1;  //why??

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates 
      //         and initialize state.
       cout << "EKF : First measurement RADAR" << endl;

      double rho = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];
      double rho_dot = measurement_pack.raw_measurements_[2];

      double x = rho * cos(phi);
      if ( x < 0.0001 ) {
        x = 0.0001;
      }
  	  double y = rho * sin(phi);
      if ( y < 0.0001 ) {
        y = 0.0001;
      }

      double vx = rho_dot * cos(phi);
  	  double vy = rho_dot * sin(phi);


      ekf_.x_(0)= x;  
      ekf_.x_(1)= y; 
      ekf_.x_(2)= vx; 
      ekf_.x_(3)= vy; 

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      cout << "EKF : First measurement LASER" << endl;
      //  Initialize state.
      

      // set the state with the initial location and zero velocity
      ekf_.x_ << measurement_pack.raw_measurements_[0], 
                 measurement_pack.raw_measurements_[1], 
                 0, 
                 0;

    }

     //record the time stamp 

     previous_timestamp_=  measurement_pack.timestamp_; 

     ekf_.Initialize(H_laser_,R_laser_, R_radar_); 
     
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   *  Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   */
   //long long elapsed_time= measurement_pack.timestamp_-previous_timestamp_;

   //cout<< "Elapsed time: "<< elapsed_time<<endl;
   float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; 
   ekf_.F_(0,2)= ekf_.F_(1,3)=dt;
   //ekf_.UpdateTransitionF(elapsed_time);

   // Noise values from the task
  double noise_ax = 9.0;
  double noise_ay = 9.0;
  double dt_2 = dt * dt; //dt^2
  double dt_3 = dt_2 * dt; //dt^3
  double dt_4 = dt_3 * dt; //dt^4
  double dt_4_4 = dt_4 / 4; //dt^4/4
  double dt_3_2 = dt_3 / 2; //dt^3/2
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4_4 * noise_ax, 0, dt_3_2 * noise_ax, 0,
           0, dt_4_4 * noise_ay, 0, dt_3_2 * noise_ay,
           dt_3_2 * noise_ax, 0, dt_2 * noise_ax, 0,
           0, dt_3_2 * noise_ay, 0, dt_2 * noise_ay;

   /*  Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
   //ekf_.UpdateProcessNoiseQ(dt,9.,9.);

   ekf_.Predict();

  /**
   * Update
   */

  /**
   * 
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    //  Radar updates
    // Extended Kalman Filter
    // void UpdateEKF(const Eigen::VectorXd &z);

    z_= VectorXd(3);

   ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
   cout<<"calculated"<<endl;
   z_(0) = measurement_pack.raw_measurements_(0) ;
   z_(1) = measurement_pack.raw_measurements_(1) ;
   z_(2) = measurement_pack.raw_measurements_(2) ;

   ekf_.UpdateEKF(z_);


  } else {
    //  Laser updates
    // void Update(const Eigen::VectorXd &z);
    z_= VectorXd(2);
    ekf_.H_ = H_laser_;

    //The measured values are in raw_measurements and should go to 
    //the measurement vector z Eigen::VectorXd z_;

    z_(0) = measurement_pack.raw_measurements_(0) ;
    z_(1) = measurement_pack.raw_measurements_(1) ;

    ekf_.Update(z_);
      
    

  }
   
  //update the previous timestamp
   previous_timestamp_= measurement_pack.timestamp_;
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
