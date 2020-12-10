#pragma once

#include "Point2d.h"

namespace cg {
// Fundamentally, 2 points define a line.
class Line2d {
 public:
  Line2d();
  Line2d(Point2d v); // Line segment from origin to point v
  Line2d(Point2d u, Point2d v); // Line segment from point u to v

  // Getters
  double a() const;
  double b() const;
  double c() const;
  Point2d vec() const;

  // Euclidean length of this vector
  double length() const;

  Line2d normalize();

  // Cross product: This vec x other vec
  double cross(const Line2d &other) const;

  // Dot product
  double dot(const Line2d &other) const;

 private:
  // Line segment bounded by 2 points
  Point2d u_, v_;
  // Vector that's independent of displacement (v-u) represented as a point
  Point2d vec_;
  double a_, b_, c_;

  // Compute coefficients of the standard line equation ax + by + c = 0
  void computeABC(const Point2d &pt1, const Point2d &pt2);
};

} // namespace cg