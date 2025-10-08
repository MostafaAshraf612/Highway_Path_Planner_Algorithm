#ifndef POLYNOMIAL_SOLVER_H
#define POLYNOMIAL_SOLVER_H

#include <vector>
#include <iostream>
#include "Eigen-3.3/Eigen/Dense"

std::vector<double> polynomial_solver(
    std::vector<double> init,
    std::vector<double> final,
    double T
);

#endif
