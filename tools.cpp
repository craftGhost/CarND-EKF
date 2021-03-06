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
  VectorXd rmse(4);
  rmse<<0, 0, 0, 0;
  
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }
  
  for (int i=0; i<estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  
  }
  
  rmse = rmse/estimations.size();
  rmse = rmse.array().sqrt();
  
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  float px2 = px*px;
  float py2 = py*py;
  float p3 = (px2+py2)*(px2+py2)*(px2+py2);
  
  if (px == 0 && py == 0) {
      std::cout << "Both px and py are zeros!\n";
      return Hj;
  }
  
  Hj << px/sqrt(px2+py2), py/sqrt(px2+py2), 0, 0,
        -(py/(px2+py2)), px/(px2+py2), 0, 0,
        py*(vx*py-vy*px)/sqrt(p3), px*(vy*px-vx*py)/sqrt(p3), px/sqrt(px2+py2), py/sqrt(px2+py2);
  
  return Hj;
}
