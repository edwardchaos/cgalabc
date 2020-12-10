#pragma once

namespace cg {

// Small value for comparing double floating equality
inline const double EPS = 1e-9;

struct Point2d {
  double x, y;

  Point2d();
  Point2d(double x, double y);

  Point2d operator-(const Point2d &other) const;
  bool operator==(const Point2d &other) const;

  void print() const;
};

} // namespace cg