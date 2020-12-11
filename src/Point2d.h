#pragma once

namespace cg {

// Small value for comparing double floating equality
inline const double EPS = 1e-9;

class Point2d {
 public:
  Point2d();
  Point2d(double x, double y);

  // Getters
  double x() const;
  double y() const;

  Point2d operator-(const Point2d &other) const;
  bool operator==(const Point2d &other) const;
  bool operator!=(const Point2d &other) const;

  double dist(const Point2d &other) const;
  void print() const;

 private:
  double x_, y_;
};

} // namespace cg