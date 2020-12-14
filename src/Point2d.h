#pragma once

#include <cstddef>
#include <functional>
#include <memory>

namespace cg {
class Point2d;
typedef std::shared_ptr<Point2d> Point2d_ptr;

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

struct hashPoint2d{
  std::size_t operator()(Point2d const&pt)const{
    std::size_t h = 17;
    h = h * 31 + std::hash<double>()(pt.x());
    h = h * 31 + std::hash<double>()(pt.y());
    return h;
  }
};

} // namespace cg