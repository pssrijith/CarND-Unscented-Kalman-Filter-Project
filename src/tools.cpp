#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse << 0.0, 0.0, 0.0, 0.0;

    int est_size = estimations.size();

    if (est_size == 0) {
        cout << "Error: Estimations vector cannot be a 0 length vector" << endl;
        return rmse;
    }

    if (est_size != ground_truth.size()) {
        cout << "Error: Estimations and ground_truth vectors must have same length"
            << endl;
        return rmse;
    }

    for(int i=0; i<estimations.size(); i++) {
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    rmse = rmse / estimations.size();
    // root mean rmse for each of (px, py, vx, vy)
    rmse = rmse.array().sqrt();

    return rmse;

}
