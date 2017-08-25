#ifndef polynomial_H
#define polynomial_H

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

class Polynomial {
public:
  
  // Constructor
  Polynomial();
  
  // Destructor
  virtual ~Polynomial();
  
  Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
  
  double polyeval(Eigen::VectorXd coeffs, double x);
  
};

#endif /* polynomial_H */
