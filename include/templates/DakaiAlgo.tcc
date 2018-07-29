

template <typename function_type>
double partialDerivative(const double& point, function_type objective_func, const Variables& v)
{
  double dx1;
  v.getParam("derivative_epsilon", dx1);
  const double dx2 = dx1 * 2;
  const double dx3 = dx1 * 3;

  const double m1 = (objective_func(point + dx1) - objective_func(point - dx1)) / 2;
  const double m2 = (objective_func(point + dx2) - objective_func(point - dx2)) / 4;
  const double m3 = (objective_func(point + dx3) - objective_func(point - dx3)) / 6;

  const double fifteen_m1 = 15 * m1;
  const double six_m2 = 6 * m2;
  const double ten_dx1 = 10 * dx1;

  return ((fifteen_m1 - six_m2) + m3) / ten_dx1;
};

template <typename function_type>
double fullDerivative(const Eigen::Vector2d& point, function_type objective_func,
                      const Variables& v)  // function to compute full derivative of the 2-variable function
{
  double dx1;
  v.getParam("derivative_epsilon", dx1);
  const double dx2 = dx1 * 2;
  const double dx3 = dx1 * 3;

  double m1 = (objective_func(point(0, 0) + dx1, point(1, 0)) - objective_func(point(0, 0) - dx1, point(1, 0))) / 2.0;
  double m2 = (objective_func(point(0, 0) + dx2, point(1, 0)) - objective_func(point(0, 0) - dx2, point(1, 0))) / 4.0;
  double m3 = (objective_func(point(0, 0) + dx3, point(1, 0)) - objective_func(point(0, 0) - dx3, point(1, 0))) / 6.0;

  double fifteen_m1 = 15 * m1;
  double six_m2 = 6 * m2;
  double ten_dx1 = 10 * dx1;

  double answer = ((fifteen_m1 - six_m2) + m3) / ten_dx1;

  m1 = (objective_func(point(0, 0), point(1, 0) + dx1) - objective_func(point(0, 0), point(1, 0) - dx1)) / 2.0;
  m2 = (objective_func(point(0, 0), point(1, 0) + dx2) - objective_func(point(0, 0), point(1, 0) - dx2)) / 4.0;
  m3 = (objective_func(point(0, 0), point(1, 0) + dx3) - objective_func(point(0, 0), point(1, 0) - dx3)) / 6.0;

  fifteen_m1 = 15 * m1;
  six_m2 = 6 * m2;
  ten_dx1 = 10 * dx1;

  answer += ((fifteen_m1 - six_m2) + m3) / ten_dx1;

  return answer;
}