#pragma once

#include <vector>
#include <cmath>

class Dubins {
  
  public:

    // constructor and distructor
    Dubins(std::vector<double> start, std::vector<double> end, double Kmax);
    ~Dubins();
    
    // utils
    double mod2pi(double angle);
    std::vector<double> scale_to_standard(double x0, double y0, double th0, double xf, double yf, double thf, double kmax);
    std::vector<double> scale_from_standard(double lam, double sc_s1, double sc_s2, double sc_s3);
    double sinc(double x);

    // dubins curves cases
    std::vector<double> d_lsl(double sc_th0, double sc_thf, double sc_kmax);
    std::vector<double> d_rsr(double sc_th0, double sc_thf, double sc_kmax);
    std::vector<double> d_lsr(double sc_th0, double sc_thf, double sc_kmax);
    std::vector<double> d_rsl(double sc_th0, double sc_thf, double sc_kmax);
    std::vector<double> d_rlr(double sc_th0, double sc_thf, double sc_kmax);
    std::vector<double> d_lrl(double sc_th0, double sc_thf, double sc_kmax);

  private:
    std::vector<double> start;
    std::vector<double> end;
    double Kmax;

};