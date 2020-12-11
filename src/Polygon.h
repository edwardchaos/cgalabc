#pragma once

#include <vector>
#include "type.h"

namespace cg{

/*
 * Vertices of the polygon are ordered counter clockwise (cross product is
 * positive). Last point is equal to the first point.
 */
class Polygon{
 public:
  Polygon();

  // Input is counter clockwise ordered points. If the last point is not yet
  // equal to first point (to close the polygon), the first point is appended
  // to close the polygon.
  Polygon(std::vector<Point2d> &points);
  Polygon(std::initializer_list<Point2d> points);

  bool verifyPolygon() const;

  // getters
  std::vector<Point2d> vertices();

  bool isConvex() const;

  double area() const;

  bool containsPoint(const Point2d &pt) const;

 private:
  std::vector<Point2d> vertices_;

};
} // namespace cg