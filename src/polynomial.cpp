#include "polynomial.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

Polynomial::Polynomial() {}

Polynomial::~Polynomial() {}

/**
 Fits a polynomial to a set of coordinate points
 Adapted from: https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716

 @param xvals x coordinate points
 @param yvals y coordinate points
 @param order polynomial order
 @return polynomial coefficients
 */
Eigen::VectorXd Polynomial::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);
  
  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }
  
  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }
  
  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

/**
 Evaluates a polynomial at a point x

 @param coeffs polynomial coefficients
 @param x point at which the polynomial is evaluated
 @return value y at point x
 */
double Polynomial::polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}
