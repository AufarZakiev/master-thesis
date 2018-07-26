template <typename function_type>
double partialDerivative(const double& x, function_type func, const Variables& v)
{
  double dx1;
  v.getParam("derivative_epsilon", dx1);
  const double dx2 = dx1 * 2;
  const double dx3 = dx1 * 3;

  const double m1 = (func(x + dx1) - func(x - dx1)) / 2;
  const double m2 = (func(x + dx2) - func(x - dx2)) / 4;
  const double m3 = (func(x + dx3) - func(x - dx3)) / 6;

  const double fifteen_m1 = 15 * m1;
  const double six_m2 = 6 * m2;
  const double ten_dx1 = 10 * dx1;

  return ((fifteen_m1 - six_m2) + m3) / ten_dx1;
};