#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
   int n= estimations.size();
   float sum=0.0;

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
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
}
