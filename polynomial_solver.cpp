#include "polynomial_solver.h"
using namespace std;

vector<double> polynomial_solver(vector<double> start , vector<double> end , double T) {
    /*
      inputs:
        - start point (si , si_dot , si_ddot)
        - end point   (sf , sf_dot , sf_ddot)
      outputs:
        - a vector of 6 doubles containing coefficients
          of the jerk-minimizing quintic polynomial
    */

    vector<double> coefficients(6); // Reserve space for 6 coefficients
    coefficients[0] = start[0];           // a0
    coefficients[1] = start[1];           // a1
    coefficients[2] = start[2] / 2.0;     // a2 = s_ddot/2

    Eigen::MatrixXd A_matrix(3,3);
    A_matrix << pow(T,3),     pow(T,4),      pow(T,5),
                3*pow(T,2),   4*pow(T,3),    5*pow(T,4),
                6*T,          12*pow(T,2),   20*pow(T,3);

    double c1 = coefficients[0] + coefficients[1]*T + coefficients[2]*pow(T,2);
    double c2 = coefficients[1] + 2*coefficients[2]*T;
    double c3 = 2 * coefficients[2];

    Eigen::VectorXd S_Matrix(3);
    S_Matrix << end[0] - c1,
                end[1] - c2,
                end[2] - c3;

    //Eigen::VectorXd res = A_matrix.inverse() * S_Matrix;
    // solve with a stable decomposition
    Eigen::VectorXd res = A_matrix.colPivHouseholderQr().solve(S_Matrix);
    coefficients[3] = res[0];  // a3
    coefficients[4] = res[1];  // a4
    coefficients[5] = res[2];  // a5

    return coefficients;
}