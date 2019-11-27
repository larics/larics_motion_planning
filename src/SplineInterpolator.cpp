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
  for (int i=1; i<6; i++){ 
    t(i) = t(i-1)*duration;
  }

  // Calculate spline coefficients
  Eigen::VectorXd coefficients = getSplineOrder5Coefficients(conditions, t);
  //trajectory_ = sampleTrajectory(coefficients, duration, sample_time);
  // Get time scaling factor
  double s = calculateTimeScalingFactor(coefficients, constraints, duration, 
    sample_time);

  while (fabs(s-1.0) > 0.01){
    duration *= s;
    t(0) = 1.0;
    for (int i=1; i<6; i++){ 
      t(i) = t(i-1)*duration;
    }
    coefficients = getSplineOrder5Coefficients(conditions, t);
    s = calculateTimeScalingFactor(coefficients, constraints, duration, 
    sample_time);
  }
  trajectory_ = sampleTrajectory(coefficients, duration, sample_time);

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
  coefficients(3) = -(20*p0 - 20*p1 + 12*t(1)*v0 + 8*t(1)*v1 - t(2)*a1 + 3*t(2)*a0)/(2*t(3));
  coefficients(4) = (30*p0 - 30*p1 + 16*t(1)*v0 + 14*t(1)*v1 - 2*t(2)*a1 + 3*t(2)*a0)/(2*t(4));
  coefficients(5) = -(12*p0 - 12*p1 + 6*t(1)*v0 + 6*t(1)*v1 - t(2)*a1 + t(2)*a0)/(2*t(5));

  return coefficients;
}

inline Eigen::VectorXd SplineInterpolator::calculatePolynomialValueOrder5(
  Eigen::VectorXd coefficients, double time)
{
  int n = coefficients.size();
  Eigen::VectorXd t(n); t(0) = 1.0;

  for (int i=1; i<n; i++){ 
    t(i) = t(i-1)*time;
  }

  // Values are position and 4 derivatives of position
  Eigen::VectorXd values(5);
  values(0) = coefficients.transpose()*t;
  for (int i=0; i<4; i++){
    int order = n-i;
    for (int j=n-1; j>=0; j--){
      coefficients(j) *= double(j-i);
    }
    for (int j=0; j<=i; j++){
      coefficients(j) = 0.0;
    }

    for (int j=t.size()-1; j>0; j--){
      t(j) = t(j-1);
    }
    t(0) = 0;

    values(i+1) = coefficients.transpose()*t;
  }
  return values;
}

Trajectory SplineInterpolator::sampleTrajectory(Eigen::VectorXd coefficients, 
  double duration, double sample_time)
{
  Trajectory trajectory;
  int n = int(round(duration/sample_time)) + 1; // +1 for last data point
  //cout << duration << " " << sample_time << " " << n << endl;
  trajectory.position.resize(n,1);
  trajectory.velocity.resize(n,1);
  trajectory.acceleration.resize(n,1);
  trajectory.jerk.resize(n,1);
  trajectory.split.resize(n,1);
  trajectory.time.resize(n);
  //cout << trajectory.position << endl;
  //cout << calculatePolynomialValueOrder5(coefficients, 0.0) << endl;

  double t = 0.0;
  for (int i=0; i<n; i++){
    Eigen::VectorXd current_point = calculatePolynomialValueOrder5(
      coefficients, t);
    //cout << calculatePolynomialValueOrder5(coefficients, t) << endl;

    trajectory.position(i,0) = 1.0; //current_point(0);
    trajectory.velocity(i,0) = current_point(1);
    trajectory.acceleration(i,0) = current_point(2);
    trajectory.jerk(i,0) = current_point(3);
    trajectory.split(i,0) = current_point(4);
    trajectory.time(i) = t;

    t += sample_time;
  }

  return trajectory;
}

double SplineInterpolator::calculateTimeScalingFactor(
  Eigen::VectorXd coefficients, Eigen::VectorXd constraints, 
  double duration, double sample_time)
{
  double s = 1.0;

  Eigen::VectorXd max_dynamic(constraints.size());
  trajectory_ = sampleTrajectory(coefficients, duration, sample_time);
  max_dynamic(0) = max(trajectory_.velocity.maxCoeff(), fabs(
    trajectory_.velocity.minCoeff()));
  max_dynamic(1) = max(trajectory_.acceleration.maxCoeff(), fabs(
    trajectory_.acceleration.minCoeff()));

  s = max(max_dynamic(0)/constraints(0), sqrt(max_dynamic(1)/constraints(1)));

  return s;
}