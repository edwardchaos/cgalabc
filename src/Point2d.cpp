#include "Point2d.h"

#include <cmath>
#include <stdio.h>

namespace cg{
Point2d::Point2d():x(0),y(0){}
Point2d::Point2d(double x, double y) : x(x), y(y){}

Point2d Point2d::operator-(const Point2d &other) const {
  return {x - other.x, y - other.y};
}

bool Point2d::operator==(const Point2d &other) const{
  return (fabs(x - other.x)) < EPS && (fabs(y - other.y) < EPS);
}

void Point2d::print() const{
  printf("Point: (%.3f, %.3f)\n", x, y);
}

} //namespace cg