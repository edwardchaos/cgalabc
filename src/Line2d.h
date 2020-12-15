#pragma once

#include <memory>

#include "Point2d.h"

namespace cg {

class Line2d;
typedef std::shared_ptr<Line2d> Line2d_ptr;

// Fundamentally, 2 points define a line.
class Line2d {
 public:
  Line2d();
  Line2d(Point2d v); // Line segment from origin to point v
  Line2d(Point2d u, Point2d v); // Line segment from point u to v
  Line2d(double a, double b, double c);

  // Getters
  double a() const;
  double b() const;
  double c() const;
  Point2d vec() const;
  Point2d pt1() const;
  Point2d pt2() const;

  // Euclidean length of this vector
  double length() const;

  /*
   * Returns a Line2d formed by the points of a unit vector in the same
   * direction as the calling line. Line is unit vector with base at the origin.
   */
  Line2d normalize();

  // Cross product aka wedge product: This vec x other vec
  double cross(const Line2d &other) const;

  // Dot product
  double dot(const Line2d &other) const;

 private:
  // Line segment bounded by 2 points
  Point2d u_, v_;
  // Vector that's independent of displacement (v-u) represented as a point
  Point2d vec_;
  // Compute coefficients of the standard line equation ax + by + c = 0
  double a_, b_, c_;
  void computeABC(const Point2d &pt1, const Point2d &pt2);
};

} // namespace cg