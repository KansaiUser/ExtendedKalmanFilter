#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;


  //Eigen::MatrixXd F_;
  // F = MatrixXd(2, 2);
  // F << 1, 1, 0, 1;
  F_ = MatrixXd(4,4);
  F_ << 1., 0., 1., 0.,
        0., 1., 0., 1.,
        0., 0., 1., 0., 
        0., 0., 0., 1.;

  // process covariance matrix
  //Eigen::MatrixXd Q_;   
  //These are just initialization values. They shuld be updated
  Q_ = MatrixXd(4,4);
  Q_ <<   1., 0., 1., 0.,
          0., 1., 0., 1.,
          1., 0., 1., 0., 
          0., 1., 0., 1.;



}

void KalmanFilter::Predict() {
   // predict the state
   
  //self.x= self.F*self.x   + self.u 
   x_ =F_*x_;   // not u (external force)??
  //self.P = self.F * self.P * np.transpose(self.F)
  P_=F_*P_*F_.transpose()+ Q_;

  // KF Prediction step
   // x=F*x+u;
  //  P=F*P*F.transpose()+ Q;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}

 void KalmanFilter::UpdateTransitionF(long long elapsed_time){
   
   F_(0,2)= elapsed_time;   //TODO verify type
   F_(1,3) = elapsed_time;

 }
 void KalmanFilter::UpdateProcessNoiseQ(long long dt,float ax, float ay){
   // Modify Q
  
   long long dt4_4 = (dt*dt*dt*dt)/4;
   long long dt3_2=  (dt*dt*dt)/2;
   long long dt2= dt*dt;
    
  Q_(0,0) =dt4_4 * ax*ax; 
  Q_(0,2) =dt3_2 * ax*ax;
  Q_(1,1) =dt4_4 * ay*ay;
  Q_(1,3) =dt3_2 * ay*ay;

  Q_(2,0) =dt3_2 * ax*ax;
  Q_(2,2) =dt2 * ax*ax;
  Q_(3,1) =dt3_2 * ay*ay;
  Q_(3,3) =dt2 * ay*ay;


 }