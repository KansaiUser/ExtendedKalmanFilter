#include "kalman_filter.h"
#include <math.h>
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Initialize(MatrixXd &H_laser_in,MatrixXd &R_laser_in,Eigen::MatrixXd &R_radar_in){

// x and P are set 

  H_=H_laser_in;
  R_laser_= R_laser_in;
  R_radar_= R_radar_in;

  //Initial transition matrix F_
  F_ = MatrixXd(4,4);
  F_ << 1., 0., 1., 0.,
        0., 1., 0., 1.,
        0., 0., 1., 0., 
        0., 0., 0., 1.;

 //These are just initialization values. They shuld be updated
  Q_ = MatrixXd(4,4);
  Q_ <<   1., 0., 1., 0.,
          0., 1., 0., 1.,
          1., 0., 1., 0., 
          0., 1., 0., 1.;


}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  //R_ = R_in;
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
  x_ =F_*x_;   // not u (external force)??
  P_=F_*P_*F_.transpose()+ Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  
  //update the state by using Kalman Filter equations
  
//We have the observations z
// We also here use the H_laser that in our case is just H_
  //We have a conversion of the position based on our x
  VectorXd y;
  MatrixXd S;
  MatrixXd K;

  y=z-H_*x_;  // the error
  //other= H_* x_;
  S=H_*P_*H_.transpose()+R_laser_;
  K=P_*H_.transpose()*S.inverse();

  //Now the new estimate updates

  x_=x_+K*y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_=(I -K*H_)*P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

// z has three components   
//  the Jacobian matrix H_jH is used to calculate  S, K and P

// We need to use some h(x) to convert from x_ to the necessary term to substract 
// from z. which is of the form (ro, th, ro dot)
  VectorXd y;
  MatrixXd S;
  MatrixXd K;
//Calculate y  we use the equations that map x' to polar coordinates
//  y= z - H*X so we need to convert these x to polar 
float px=x_(0);
float py=x_(1);

float vx = x_(2);
float vy = x_(3);

VectorXd dif=VectorXd(3);
dif(0)=sqrt(px*px+py*py);
dif(1)=atan2(py,px);
dif(2)=(px*vx+py*vy)/(sqrt(px*px+py*py));  //BE CAREFUL THIS COULD BE 0!!


y=z-dif;

//std::cout<<"uno"<<std::endl;

 while ( y(1) > M_PI || y(1) < -M_PI ) {
    if ( y(1) > M_PI ) {
      y(1) -= M_PI;
    } else {
      y(1) += M_PI;
    }
  }
//std::cout<<"dos"<<std::endl;

 //MatrixXd Ht = H_.transpose();
//std::cout<<"tres"<<std::endl;
//Replacement
  S=H_*P_*H_.transpose()+R_radar_;

//S=S+R_laser;

//std::cout<<"cuatro"<<std::endl;

  K=P_*H_.transpose()*S.inverse();

//std::cout<<"cinco"<<std::endl;
  //Now the new estimate updates

  x_=x_+K*y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_=(I -K*H_)*P_;

//not from here
/*
MatrixXd Hj_;
Hj_= tools.CalculateJacobian(x_);


//Calculate S (Jacobian)
S=Hj_*P_*Hj_.transpose()+R_radar_;


//Calcualte K(Jacobian)

K=P_*Hj_.transpose()*S.inverse();

//calculate X

x_=x_+K*y;


//calcualte P  (Jacobian)
 long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_=(I -K*Hj_)*P_;
*/

}

 

 void KalmanFilter::UpdateTransitionF(long long elapsed_time){
   
   F_(0,2)= elapsed_time;   //TODO verify type
   F_(1,3) = elapsed_time;

 }
 void KalmanFilter::UpdateProcessNoiseQ(float dt,float ax, float ay){
   // Modify Q
  
   long long dt4_4 = (dt*dt*dt*dt)/4;
   long long dt3_2=  (dt*dt*dt)/2;
   long long dt2= dt*dt;
    
  Q_(0,0) =dt4_4 * ax;//*ax; 
  Q_(0,2) =dt3_2 * ax;//*ax;
  Q_(1,1) =dt4_4 * ay;//*ay;
  Q_(1,3) =dt3_2 * ay;//*ay;

  Q_(2,0) =dt3_2 * ax;//*ax;
  Q_(2,2) =dt2 * ax;//*ax;
  Q_(3,1) =dt3_2 * ay;//*ay;
  Q_(3,3) =dt2 * ay;//*ay;


 }