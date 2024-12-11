#pragma once

#include <vector>
#include <cmath>  
#include <cstdlib>
#include <iostream>

#include "graph_generator/sampling_based/utils/rrt.hpp"

struct dubins_arc
{
  double x0;
  double y0;
  double th0;
  double L;
  double k;
  double xf;
  double yf;
  double thf;
};

struct dubins_curve
{
  struct dubins_arc a1;
  struct dubins_arc a2;
  struct dubins_arc a3;
  double L;
};

class Dubins
{

public:
  // constructor and distructor
  Dubins(std::vector<double> start, std::vector<double> end, double Kmax);
  ~Dubins();

  // utils
  double mod2pi(double angle);
  std::vector<double> scale_to_standard(double x0, double y0, double th0, double xf, double yf, double thf, double kmax);
  std::vector<double> scale_from_standard(double lam, double sc_s1, double sc_s2, double sc_s3);
  double sinc(double x);
  std::vector<double> circline(double L, double x0, double y0, double th0, double k);

  // dubins curves cases
  std::vector<double> d_lsl(double sc_th0, double sc_thf, double sc_kmax);
  std::vector<double> d_rsr(double sc_th0, double sc_thf, double sc_kmax);
  std::vector<double> d_lsr(double sc_th0, double sc_thf, double sc_kmax);
  std::vector<double> d_rsl(double sc_th0, double sc_thf, double sc_kmax);
  std::vector<double> d_rlr(double sc_th0, double sc_thf, double sc_kmax);
  std::vector<double> d_lrl(double sc_th0, double sc_thf, double sc_kmax);

  // main
  struct dubins_curve dubins_shortest_path(double x0, double y0, double th0, double xf, double yf, double thf, double kmax, RRT &_rrt, boost::geometry::model::multi_polygon<polygon_t> &map);
  // helper
  std::vector<std::vector<double> (Dubins::*)(double, double, double)> function_vector;
  std::vector<double> call_function(int index, double sc_th0, double sc_thf, double sc_kmax);
  struct dubins_curve build_dubins(double x0, double y0, double th0, double s1, double s2, double s3, double k1, double k2, double k3);
  struct dubins_arc build_dubins_arc(double x0, double y0, double th0, double k, double L);

  // multipoint dubins
  std::vector<struct dubins_curve> dubins_multi_point(double x0, double y0, double th0, double xf, double yf, double thf, std::vector<std::vector<double>> points, double kmax, RRT &_rrt, boost::geometry::model::multi_polygon<polygon_t> &map);
  bool valid_curve(struct dubins_curve curve, RRT &_rrt, boost::geometry::model::multi_polygon<polygon_t> &map);


private:
  std::vector<double> start;
  std::vector<double> end;
  double Kmax;
};