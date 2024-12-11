#include "graph_generator/utils/dubins.hpp"

Dubins::Dubins(std::vector<double> start, std::vector<double> end, double Kmax)
{
  this->start = start;
  this->end = end;
  this->Kmax = Kmax;
  function_vector = {
      &Dubins::d_lsl,
      &Dubins::d_rsr,
      &Dubins::d_lsr,
      &Dubins::d_rsl,
      &Dubins::d_rlr,
      &Dubins::d_lrl};
}

Dubins::~Dubins() {}

double Dubins::mod2pi(double angle)
{
  // double out = angle;
  // while (out < 0)
  // {
  //   out = out + 2 * M_PI;
  // }
  // while (out >= 2 * M_PI)
  // {
  //   out = out - 2 * M_PI;
  // }
  // return out;
  double result = std::fmod(angle, 2 * M_PI);
  if (result < 0)
  {
    result += 2 * M_PI;
  }
  return result;
}

std::vector<double> Dubins::scale_to_standard(double x0, double y0, double th0, double xf, double yf, double thf, double kmax)
{
  double dx = xf - x0;
  double dy = yf - y0;
  double phi = atan2(dy, dx);
  double lam = hypot(dx, dy) / 2;
  std::vector<double> result;
  result.push_back(this->mod2pi(th0 - phi)); // scaled th0
  result.push_back(this->mod2pi(thf - phi)); // scaled thf
  result.push_back(kmax * lam);              // scaled kmax
  result.push_back(lam);                     // scaled factor

  return result;
}

std::vector<double> Dubins::scale_from_standard(double lam, double sc_s1, double sc_s2, double sc_s3)
{
  std::vector<double> result;
  result.push_back(lam * sc_s1); // s1
  result.push_back(lam * sc_s2); // s2
  result.push_back(lam * sc_s3); // s3
  return result;
}

double Dubins::sinc(double x)
{
  if (abs(x) < 0.002)
  {
    return 1 - pow(x, 1 / 3) * (1 - pow(x, 1 / 10));
  }
  else
  {
    return sin(x) / x;
  }
}

std::vector<double> Dubins::d_lsl(double sc_th0, double sc_thf, double sc_kmax)
{
  double radious = 1 / sc_kmax;
  double c = cos(sc_thf) - cos(sc_th0);
  double s = 2 * sc_kmax + sin(sc_th0) - sin(sc_thf);
  double temp1 = atan2(c, s);
  double sc_s1 = radious * this->mod2pi(temp1 - sc_th0);
  double temp2 = 2 + 4 * pow(sc_kmax, 2) - 2 * cos(sc_th0 - sc_thf) + 4 * sc_kmax * (sin(sc_th0) - sin(sc_thf));
  if (temp2 < 0)
  {
    return {0, 0, 0};
  }
  double sc_s2 = radious * sqrt(temp2);
  double sc_s3 = radious * this->mod2pi(sc_thf - temp1);
  return {sc_s1, sc_s2, sc_s3};
}

std::vector<double> Dubins::d_rsr(double sc_th0, double sc_thf, double sc_kmax)
{
  double radious = 1 / sc_kmax;
  double c = cos(sc_th0) - cos(sc_thf);
  double s = 2 * sc_kmax - sin(sc_th0) + sin(sc_thf);
  double temp1 = atan2(c, s);
  double sc_s1 = radious * this->mod2pi(sc_th0 - temp1);
  double temp2 = 2 + 4 * pow(sc_kmax, 2) - 2 * cos(sc_th0 - sc_thf) - 4 * sc_kmax * (sin(sc_th0) - sin(sc_thf));
  if (temp2 < 0)
  {
    return {0, 0, 0};
  }
  double sc_s2 = radious * sqrt(temp2);
  double sc_s3 = radious * this->mod2pi(temp1 - sc_thf);
  return {sc_s1, sc_s2, sc_s3};
}

std::vector<double> Dubins::d_lsr(double sc_th0, double sc_thf, double sc_kmax)
{
  double radious = 1 / sc_kmax;
  double c = cos(sc_th0) + cos(sc_thf);
  double s = 2 * sc_kmax + sin(sc_th0) + sin(sc_thf);
  double temp1 = atan2(-c, s);
  double temp3 = 4 * pow(sc_kmax, 2) - 2 + 2 * cos(sc_th0 - sc_thf) + 4 * sc_kmax * (sin(sc_th0) + sin(sc_thf));
  if (temp3 < 0)
  {
    return {0, 0, 0};
  }
  double sc_s2 = radious * sqrt(temp3);
  double temp2 = -atan2(-2, sc_s2 * sc_kmax);
  double sc_s1 = radious * this->mod2pi(temp1 + temp2 - sc_th0);
  double sc_s3 = radious * this->mod2pi(temp1 + temp2 - sc_thf);
  return {sc_s1, sc_s2, sc_s3};
}

std::vector<double> Dubins::d_rsl(double sc_th0, double sc_thf, double sc_kmax)
{
  double radious = 1 / sc_kmax;
  double c = cos(sc_th0) + cos(sc_thf);
  double s = 2 * sc_kmax - sin(sc_th0) - sin(sc_thf);
  double temp1 = atan2(c, s);
  double temp3 = 4 * pow(sc_kmax, 2) - 2 + 2 * cos(sc_th0 - sc_thf) - 4 * sc_kmax * (sin(sc_th0) + sin(sc_thf));
  if (temp3 < 0)
  {
    return {0, 0, 0};
  }
  double sc_s2 = radious * sqrt(temp3);
  double temp2 = atan2(2, sc_s2 * sc_kmax);
  double sc_s1 = radious * this->mod2pi(sc_th0 - temp1 + temp2);
  double sc_s3 = radious * this->mod2pi(sc_thf - temp1 + temp2);
  return {sc_s1, sc_s2, sc_s3};
}

std::vector<double> Dubins::d_rlr(double sc_th0, double sc_thf, double sc_kmax)
{
  double radious = 1 / sc_kmax;
  double c = cos(sc_th0) - cos(sc_thf);
  double s = 2 * sc_kmax - sin(sc_th0) + sin(sc_thf);
  double temp1 = atan2(c, s);
  double temp2 = 0.125 * (6 - 4 * pow(sc_kmax, 2) + 2 * cos(sc_th0 - sc_thf) + 4 * sc_kmax * (sin(sc_th0) - sin(sc_thf)));
  if (temp2 < 0)
  {
    return {0, 0, 0};
  }
  double sc_s2 = radious * this->mod2pi(2 * M_PI - acos(temp2));
  double sc_s1 = radious * this->mod2pi(sc_th0 - temp1 + 0.5 * sc_s2 * sc_kmax);
  double sc_s3 = radious * this->mod2pi(sc_th0 - sc_thf + sc_kmax * (sc_s2 - sc_s1));
  return {sc_s1, sc_s2, sc_s3};
}

std::vector<double> Dubins::d_lrl(double sc_th0, double sc_thf, double sc_kmax)
{
  double radious = 1 / sc_kmax;
  double c = cos(sc_thf) - cos(sc_th0);
  double s = 2 * sc_kmax + sin(sc_th0) - sin(sc_thf);
  double temp1 = atan2(c, s);
  double temp2 = 0.125 * (6 - 4 * pow(sc_kmax, 2) + 2 * cos(sc_th0 - sc_thf) - 4 * sc_kmax * (sin(sc_th0) - sin(sc_thf)));
  if (temp2 < 0)
  {
    return {0, 0, 0};
  }
  double sc_s2 = radious * this->mod2pi(2 * M_PI - acos(temp2));
  double sc_s1 = radious * this->mod2pi(temp1 - sc_th0 + 0.5 * sc_s2 * sc_kmax);
  double sc_s3 = radious * this->mod2pi(sc_thf - sc_th0 + sc_kmax * (sc_s2 - sc_s1));
  return {sc_s1, sc_s2, sc_s3};
}

std::vector<double> Dubins::call_function(int index, double sc_th0, double sc_thf, double sc_kmax)
{
  return (this->*function_vector[index])(sc_th0, sc_thf, sc_kmax);
}

struct dubins_curve Dubins::build_dubins(double x0, double y0, double th0, double s1, double s2, double s3, double k1, double k2, double k3)
{
  struct dubins_arc a1 = this->build_dubins_arc(x0, y0, th0, k1, s1);
  struct dubins_arc a2 = this->build_dubins_arc(a1.xf, a1.yf, a1.thf, k2, s2);
  struct dubins_arc a3 = this->build_dubins_arc(a2.xf, a2.yf, a2.thf, k3, s3);

  struct dubins_curve result = {a1, a2, a3, a1.L + a2.L + a3.L};

  return result;
}

struct dubins_arc Dubins::build_dubins_arc(double x0, double y0, double th0, double k, double L)
{
  std::vector<double> end = this->circline(L, x0, y0, th0, k);

  struct dubins_arc result = {x0, y0, th0, L, k, end.at(0), end.at(1), end.at(2)};

  return result;
}

std::vector<double> Dubins::circline(double L, double x0, double y0, double th0, double k)
{
  double x, y, th;
  x = x0 + L * this->sinc(k * L / 2) * cos(th0 + L * k / 2);
  y = y0 + L * this->sinc(k * L / 2) * sin(th0 + L * k / 2);
  th = this->mod2pi(th0 + L * k);

  return {x, y, th};
}

struct dubins_curve Dubins::dubins_shortest_path(double x0, double y0, double th0, double xf, double yf, double thf, double kmax)
{
  std::vector<double> scaled_quantities = this->scale_to_standard(x0, y0, th0, xf, yf, thf, kmax);
  int pidx = -1;
  std::vector<double> best_dubins;
  double L = 999;
  double Lcur = 999;
  std::vector<int> ksign = {1, 0, 1, -1, 0, -1, 1, 0, -1, -1, 0, 1 - 1, 1, -1, 1, -1, 1};

  struct dubins_curve result;

  for (int i = 0; i < 6; i++)
  {
    std::vector<double> result_d = this->call_function(i, scaled_quantities.at(0), scaled_quantities.at(1), scaled_quantities.at(2));
    Lcur = result_d.at(0) + result_d.at(1) + result_d.at(2);
    if (!(result_d.at(0) == 0 && result_d.at(1) == 0 && result_d.at(2) == 0) && Lcur < L)
    {
      best_dubins = result_d;
      Lcur = L;
      pidx = i;
      break;
    }
  }

  if (pidx >= 0)
  {

    std::vector<double> s = this->scale_from_standard(scaled_quantities.at(3), best_dubins.at(0), best_dubins.at(1), best_dubins.at(2));

    result = this->build_dubins(x0, y0, th0, s.at(0), s.at(1), s.at(2), ksign.at(3 * pidx) * kmax, ksign.at(3 * pidx + 1) * kmax, ksign.at(3 * pidx + 2) * kmax);
  }

  return result;
}

std::vector<struct dubins_curve> Dubins::dubins_multi_point(double x0, double y0, double th0, double xf, double yf, double thf, std::vector<std::vector<double>> points, double kmax)
{
  std::vector<struct dubins_curve> curves;
  std::vector<double> known_point = {xf, yf, thf};

  for (auto it = points.rbegin(); it != points.rend(); ++it)
  {
    const auto &p = *it;
    double L = 999;
    double Lcur;
    struct dubins_curve best_curve;

    // find best
    for (uint deg = 0; deg < 360; deg++)
    {
      double rad = deg * M_PI / 180;
      auto aux_curve = this->dubins_shortest_path(p.at(0), p.at(1), rad, known_point.at(0), known_point.at(1), known_point.at(2), kmax);
      Lcur = aux_curve.L;
      if (Lcur < L)
      {
        best_curve = aux_curve;
        L = Lcur;
      }
    }
    known_point.at(0) = best_curve.a1.x0;
    known_point.at(1) = best_curve.a1.y0;
    known_point.at(2) = best_curve.a1.th0;

    curves.insert(curves.begin(), best_curve);
  }

  // insert last
  curves.insert(curves.begin(), this->dubins_shortest_path(x0, y0, th0, known_point.at(0), known_point.at(1), known_point.at(2), kmax));

  return curves;
}
