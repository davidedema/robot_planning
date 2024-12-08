#include "grapg_generator/utils/dubins.hpp"

Dubins::Dubins(std::vector<double> start, std::vector<double> end, double Kmax)
{
  this->start = start;
  this->end = end;
  this->Kmax = Kmax;
}

Dubins::~Dubins() {}

double Dubins::mod2pi(double angle)
{
  double result = std::fmod(ang, 2 * M_PI);
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
  double s = 2 * sc_kmax + sin(sc_th0) - sin(sc_thf);
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
  double c = cos(sc_th0) - cos(sc_thf);
  double s = 2 * sc_kmax + sin(sc_th0) - sin(sc_thf);
  double temp1 = atan2(-c, s);
  double temp3 = 4 * pow(sc_kmax, 2) - 2 + 2 * cos(sc_th0 - sc_thf) + 4 * sc_kmax * (sin(sc_th0) - sin(sc_thf));
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
  double c = cos(sc_th0) - cos(sc_thf);
  double s = 2 * sc_kmax - sin(sc_th0) - sin(sc_thf);
  double temp1 = atan2(-c, s);
  double temp3 = 4 * pow(sc_kmax, 2) - 2 + 2 * cos(sc_th0 - sc_thf) - 4 * sc_kmax * (sin(sc_th0) - sin(sc_thf));
  if (temp3 < 0)
  {
    return {0, 0, 0};
  }
  double sc_s2 = radious * sqrt(temp3);
  double temp2 = -atan2(2, sc_s2 * sc_kmax);
  double sc_s1 = radious * this->mod2pi(temp1 - temp2 + sc_th0);
  double sc_s3 = radious * this->mod2pi(temp1 - temp2 + sc_thf);
  return {sc_s1, sc_s2, sc_s3};
}

std::vector<double> Dubins::d_rlr(double sc_th0, double sc_thf, double sc_kmax)
{
  double radious = 1 / sc_kmax;
  double c = cos(sc_th0) - cos(sc_thf);
  double s = 2 * sc_kmax - sin(sc_th0) + sin(sc_thf);
  double temp1 = atan2(-c, s);
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
  double temp1 = atan2(-c, s);
  double temp2 = 0.125 * (6 - 4 * pow(sc_kmax, 2) + 2 * cos(sc_th0 - sc_thf) + 4 * sc_kmax * (sin(sc_th0) - sin(sc_thf)));
  if (temp2 < 0)
  {
    return {0, 0, 0};
  }
  double sc_s2 = radious * this->mod2pi(2 * M_PI - acos(temp2));
  double sc_s1 = radious * this->mod2pi(temp1 - sc_th0 + 0.5 * sc_s2 * sc_kmax);
  double sc_s3 = radious * this->mod2pi(sc_thf - sc_th0 + sc_kmax * (sc_s2 - sc_s1));
  return {sc_s1, sc_s2, sc_s3};
}
