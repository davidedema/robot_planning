#include "graph_generator/utils/dubins.hpp"

Dubins::Dubins()
{
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

struct dubins_curve Dubins::dubins_shortest_path(double x0, double y0, double th0, double xf, double yf, double thf, double kmax, boost::geometry::model::multi_polygon<polygon_t> &map)
{
  std::vector<double> scaled_quantities = this->scale_to_standard(x0, y0, th0, xf, yf, thf, kmax);
  int pidx = -1;
  std::vector<double> best_dubins;
  bool is_valid = true;
  double L = 999;
  double Lcur = 999;
  std::vector<int> ksign = {1, 0, 1, -1, 0, -1, 1, 0, -1, -1, 0, 1, -1, 1, -1, 1, -1, 1};

  struct dubins_curve result;

  for (int i = 0; i < 6; i++)
  {
    std::vector<double> result_d = this->call_function(i, scaled_quantities.at(0), scaled_quantities.at(1), scaled_quantities.at(2));
    Lcur = result_d.at(0) + result_d.at(1) + result_d.at(2);

    struct dubins_curve tmp_curve;

    if (!(result_d.at(0) == 0 && result_d.at(1) == 0 && result_d.at(2) == 0))
    {
      std::vector<double> aux_1 = this->scale_from_standard(scaled_quantities.at(3), result_d.at(0), result_d.at(1), result_d.at(2));
      tmp_curve = this->build_dubins(x0, y0, th0, aux_1.at(0), aux_1.at(1), aux_1.at(2), ksign.at(3 * i) * kmax, ksign.at(3 * i + 1) * kmax, ksign.at(3 * i + 2) * kmax);

      if (Lcur < L && this->valid_curve(tmp_curve, map))
      {
        is_valid = true;
        best_dubins = result_d;
        L = Lcur;
        pidx = i;
      }
    }
  }
  if (pidx >= 0)
  {

    std::vector<double> s = this->scale_from_standard(scaled_quantities.at(3), best_dubins.at(0), best_dubins.at(1), best_dubins.at(2));

    result = this->build_dubins(x0, y0, th0, s.at(0), s.at(1), s.at(2), ksign.at(3 * pidx) * kmax, ksign.at(3 * pidx + 1) * kmax, ksign.at(3 * pidx + 2) * kmax);
  }

  if (!is_valid)
  {
    throw std::runtime_error("No valid Dubins curve found.");
  }

  return result;
}

std::vector<struct dubins_curve> Dubins::dubins_multi_point(double x0, double y0, double th0, double xf, double yf, double thf, std::vector<std::vector<double>> points, double kmax, boost::geometry::model::multi_polygon<polygon_t> &map)
{
  std::vector<struct dubins_curve> curves;
  std::vector<double> known_point = {xf, yf, thf};

  //? print the input
  std::cout << "x0: " << x0 << " y0: " << y0 << " th0: " << th0 << "\n";
  std::cout << "xf: " << xf << " yf: " << yf << " thf: " << thf << "\n";
  std::cout << "Points: \n";
  for (auto &p : points)
  {
    std::cout << "x: " << p.at(0) << " y: " << p.at(1) << "\n";
  }

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
      auto aux_curve = this->dubins_shortest_path(p.at(0), p.at(1), rad, known_point.at(0), known_point.at(1), known_point.at(2), kmax, map);
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
  curves.insert(curves.begin(), this->dubins_shortest_path(x0, y0, th0, known_point.at(0), known_point.at(1), known_point.at(2), kmax, map));

  return curves;
}

std::vector<KDNode_t> Dubins::segment_arc(const dubins_arc &arc, int segments)
{
  std::vector<KDNode_t> points;   // Vector to store the points
  double step = arc.L / segments; // Segment length

  for (int i = 0; i <= segments; ++i)
  {
    double s = i * step; // Distance along the arc
    KDNode_t p(2);       // Assume KDNode_t is a 2D point like std::vector<double>(2)

    if (std::abs(arc.k) < 1e-6)
    {                                        // Straight line
      p[0] = arc.x0 + s * std::cos(arc.th0); // x-coordinate
      p[1] = arc.y0 + s * std::sin(arc.th0); // y-coordinate
    }
    else
    {                                                                          // Circular arc
      double R = 1.0 / arc.k;                                                  // Radius of curvature
      p[0] = arc.x0 + R * (std::sin(arc.th0 + arc.k * s) - std::sin(arc.th0)); // x-coordinate
      p[1] = arc.y0 - R * (std::cos(arc.th0 + arc.k * s) - std::cos(arc.th0)); // y-coordinate
    }

    points.push_back(p); // Add the point to the vector
  }

  return points;
}

bool Dubins::valid_curve(struct dubins_curve curve, boost::geometry::model::multi_polygon<polygon_t> &map)
{
  // if (curve.a1.L > 0)
  // {
  //   auto points = this->segment_arc(curve.a1, 2);
  //   for (size_t i = 0; i < points.size() - 1; ++i)
  //   {
  //     auto &p1_0 = points[i];
  //     auto &p1_1 = points[i + 1];

  //     // std::cout << "Checking segment from (" << p1_0.at(0) << ", " << p1_0.at(1) << ") "
  //               // << "to (" << p1_1.at(0) << ", " << p1_1.at(1) << ")\n";

  //     if (!this->valid_segment(p1_0, p1_1, map))
  //     {
  //       // std::cout << "Segment is invalid!\n";
  //       return false;
  //     }

  //     // std::cout << "Segment is valid.\n";
  //     // std::cout << "----------------\n";
  //   }
  // }
  // if (curve.a2.L > 0)
  // {
  //   auto points = this->segment_arc(curve.a2, 2);
  //   for (size_t i = 0; i < points.size() - 1; ++i)
  //   {
  //     auto &p1_0 = points[i];
  //     auto &p1_1 = points[i + 1];

  //     // std::cout << "Checking segment from (" << p1_0.at(0) << ", " << p1_0.at(1) << ") "
  //               // << "to (" << p1_1.at(0) << ", " << p1_1.at(1) << ")\n";

  //     if (!this->valid_segment(p1_0, p1_1, map))
  //     {
  //       // std::cout << "Segment is invalid!\n";
  //       return false;
  //     }

  //     // std::cout << "Segment is valid.\n";
  //     // std::cout << "----------------\n";
  //   }
  // }
  // if (curve.a3.L > 0)
  // {
  //   auto points = this->segment_arc(curve.a3, 2);
  //   for (size_t i = 0; i < points.size() - 1; ++i)
  //   {
  //     auto &p1_0 = points[i];
  //     auto &p1_1 = points[i + 1];

  //     // std::cout << "Checking segment from (" << p1_0.at(0) << ", " << p1_0.at(1) << ") "
  //               // << "to (" << p1_1.at(0) << ", " << p1_1.at(1) << ")\n";

  //     if (!this->valid_segment(p1_0, p1_1, map))
  //     {
  //       // std::cout << "Segment is invalid!\n";
  //       return false;
  //     }

  //     // std::cout << "Segment is valid.\n";
  //     // std::cout << "----------------\n";
  //   }
  // }

  // return true;
  KDNode_t p1_0 = {curve.a1.x0, curve.a1.y0};
  KDNode_t p1_f = {curve.a1.xf, curve.a1.yf};
  KDNode_t p2_0 = {curve.a2.x0, curve.a2.y0};
  KDNode_t p2_f = {curve.a2.xf, curve.a2.yf};
  KDNode_t p3_0 = {curve.a3.x0, curve.a3.y0};
  KDNode_t p3_f = {curve.a3.xf, curve.a3.yf};

  if (this->valid_segment(p1_0, p1_f, map) && this->valid_segment(p2_0, p2_f, map) && this->valid_segment(p3_0, p3_f, map))
  {
    return true;
  }
  return false;
}

bool Dubins::valid_segment(KDNode_t &start, KDNode_t &end, boost::geometry::model::multi_polygon<polygon_t> &map)
{
  // check if a segment with start and end points whole within the polygon map
  // will approximate the segment with a polygon so the function within can be used

  point_t start_point = point_t(start.at(0), start.at(1));
  point_t end_point = point_t(end.at(0), end.at(1));
  polygon_t segment;
  this->create_inflated_polygon(start_point, end_point, 0.001, segment);
  if (boost::geometry::within(segment, map))
  {
    return true;
  }
  return false;
}

void Dubins::create_inflated_polygon(
    const point_t &p1,
    const point_t &p2,
    double epsilon,
    polygon_t &polygon)
{
  // Extract coordinates
  double x1 = boost::geometry::get<0>(p1);
  double y1 = boost::geometry::get<1>(p1);
  double x2 = boost::geometry::get<0>(p2);
  double y2 = boost::geometry::get<1>(p2);

  // Compute the direction vector (dx, dy)
  double dx = x2 - x1;
  double dy = y2 - y1;

  // Normalize the direction vector
  double length = std::sqrt(dx * dx + dy * dy);
  dx /= length;
  dy /= length;

  // Compute the perpendicular vector (px, py)
  double px = -dy * epsilon; // Perpendicular and scaled by epsilon
  double py = dx * epsilon;

  // Compute the four corners of the rectangle
  point_t corner1(x1 + px, y1 + py);
  point_t corner2(x1 - px, y1 - py);
  point_t corner3(x2 - px, y2 - py);
  point_t corner4(x2 + px, y2 + py);

  // Construct the polygon
  polygon.outer().clear();
  polygon.outer().push_back(corner1);
  polygon.outer().push_back(corner2);
  polygon.outer().push_back(corner3);
  polygon.outer().push_back(corner4);
  polygon.outer().push_back(corner1); // Close the polygon
}