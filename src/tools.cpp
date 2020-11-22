#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
   int n= estimations.size();
   //float sum=0.0;

   VectorXd rmse(4);
   rmse<<0,0,0,0;

   //First some checks
   if(estimations.size()==0)
    {    std::cout<<"Error, estimation size is 0"<<std::endl;
      return rmse;
    }
   if(estimations.size()!=ground_truth.size())
  {
      std::cout<<"Error, estimation and grund truth size is different"<<std::endl;
      return rmse;
  }

   for(int i= 0 ; i< n; i++)
   {
      VectorXd difference= estimations[i]-ground_truth[i];
      difference=difference.array()*difference.array();   
      rmse+= difference;
   }

   rmse*=(1/n);
   rmse=rmse.array().sqrt();
   
   return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Check for division 0
  if(px==0&&py==0)
  {
      std::cout<<"Error, Both x and y are zero"<<std::endl;
      return Hj;
  }

  float term=px*px+py*py;
  float sqr=sqrt(term);
  // compute the Jacobian matrix
  Hj(0,0)=px/sqr;
  Hj(0,1)=py/sqr;
  Hj(0,2)=0.;
  Hj(0,3)=0.;
  Hj(1,0)=-py/term;
  Hj(1,1)=px/term;
  Hj(1,2)=0.;
  Hj(1,3)=0.;
  Hj(2,0)=py*(vx*py-vy*px)/pow(term,(3./2));
  Hj(2,1)=px*(vy*px-vx*py)/pow(term,(3./2));
  Hj(2,2)=px/sqr;
  Hj(2,3)=py/sqr;

  return Hj;

}
