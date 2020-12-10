#include "cgalabc.hpp"

#include <cmath>
#include <stdio.h>

namespace cg{
Point2d::Point2d():x(0),y(0){}
Point2d::Point2d(double x, double y) : x(x), y(y){}

Point2d Point2d::operator-(const Point2d &other) const{
  return {x-other.x, y-other.y};
}

void Point2d::print() const{
  printf("Point: (%.3f, %.3f)\n", x, y);
}

Line2d::Line2d(Point2d v): v(v), vec(v){
  this->u = Point2d(0,0);
}

Line2d::Line2d(Point2d u, Point2d v): u(u), v(v){
  this->vec = v - u;
}

double Line2d::length() const{
  return sqrt(pow(v.x-u.x,2) + pow(v.y-u.y,2));
}

double Line2d::cross(const Line2d &other) const{
  // 2D cross product is same as determinant
  // Geometrically, it's equal to the area of the parallelogram of the 2
  // vectors
  return this->vec.x * other.vec.y - this->vec.y * other.vec.x;
}

double Line2d::dot(const Line2d &other) const{
  return this->vec.x * other.vec.x + this->vec.y * other.vec.y;
}

} // namespace cg
