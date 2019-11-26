#include <larics_motion_planning/SplineInterpolator.h>

SplineInterpolator::SplineInterpolator()
{

}

bool SplineInterpolator::generateSplineOrder5(Eigen::VectorXd conditions, 
  Eigen::VectorXd constraints, double sample_time)
{
  // Check if initial/final condions are properly provided.
  if (conditions.rows() != 6){
    cout << "Generate spline order 5." << endl;
    cout << "  Conditions must have 6 values!" << endl;
    return false;
  }
  // Check if constraints are properly provided.
  if (constraints.rows() < 1){
    cout << "Generate spline order 5." << endl;
    cout << "  Constraints must have at least one member!" << endl;
    return false;
  }

  // Assume the time and round it to nearest multiple of sample time.
  double duration = fabs(conditions(1)-conditions(0))/constraints(0);
  duration = ceil(duration/sample_time)*sample_time;

  Eigen::VectorXd t(6); t(0) = 1.0;
  for (int i=1; i<6; i++) t(i) = t(i-1)*duration;

  // Calculate spline coefficients
  Eigen::VectorXd coefficients = getSplineOrder5Coefficients(conditions, t);


  return true;
}

inline Eigen::VectorXd SplineInterpolator::getSplineOrder5Coefficients(
  Eigen::VectorXd conditions, Eigen::VectorXd t)
{
  double p0 = conditions(0);
  double p1 = conditions(1);
  double v0 = conditions(2);
  double v1 = conditions(3);
  double a0 = conditions(4);
  double a1 = conditions(5);

  Eigen::VectorXd coefficients(6);
  coefficients(0) = p0;
  coefficients(1) = v0;
  coefficients(2) = a0/2.0;
  coefficients(3) = -(20*p0 - 20*p1 + 12*t(1)*v0 + 8*t(1)*v0 - t(2)*a1 + 3*t(2)*a0)/(2*t(3));
  coefficients(4) = (30*p0 - 30*p1 + 16*t(1)*v0 + 14*t(1)*v0 - 2*t(2)*a1 + 3*t(2)*a0)/(2*t(4));
  coefficients(5) = -(12*p0 - 12*p1 + 6*t(1)*v0 + 6*t(1)*v0 - t(2)*a1 + t(2)*a0)/(2*t(5));

  return coefficients;
}