#include "Point2d.h"

#include <cmath>
#include <stdio.h>

namespace cg{
Point2d::Point2d():x_(0),y_(0){}
Point2d::Point2d(double x, double y) : x_(x), y_(y){}

double Point2d::x() const{return x_;}
double Point2d::y() const{return y_;}

Point2d Point2d::operator-(const Point2d &other) const {
  return {x_ - other.x(), y_ - other.y()};
}

Point2d Point2d::operator+(const Point2d &other) const{
  return {x_ + other.x(), y_ + other.y()};
}

template<typename T>
Point2d Point2d::operator*(T scalar)const{
  return {x_*scalar, y_*scalar};
}

template<typename T>
Point2d Point2d::operator/(T scalar)const{
  return {x_/scalar, y_/scalar};
}

bool Point2d::operator==(const Point2d &other) const{
  return (fabs(x_ - other.x())) < EPS && (fabs(y_ - other.y()) < EPS);
}

bool Point2d::operator!=(const Point2d &other) const{
  return !operator==(other);
}

void Point2d::print() const{
  printf("Point: (%.3f, %.3f)\n", x_, y_);
}

double Point2d::dist(const Point2d &other) const{
  return sqrt(pow(other.x() - x_,2) + pow(other.y() - y_,2));
}

} //namespace cg