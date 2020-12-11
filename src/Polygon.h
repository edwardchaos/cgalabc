#pragma once

#include <vector>
#include "type.h"

namespace cg{

class Polygon{
 public:
  Polygon();
  Polygon(std::vector<Point2d> points);

  // getters
  std::vector<Point2d> vertices();

  bool isConvex() const;

  double area() const;

  bool containsPoint(const Point2d &pt) const;

 private:
  std::vector<Point2d> vertices_;

};
} // namespace cg