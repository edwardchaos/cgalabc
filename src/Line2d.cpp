#include "Line2d.h"

#include <cmath>
#include <stdexcept>
#include <stdio.h>

namespace cg{

void Line2d::computeABC(const Point2d &pt1, const Point2d &pt2){
  if(fabs(pt1.y-pt2.y) < EPS){ // horizontal
    this->b = 1;
    this->a = 0;
    this->c = -pt1.y;

  }else if(fabs(pt1.x-pt2.x) < EPS){ // vertical
    this->b = 0;
    this->a = 1;
    this->c = -pt1.x;

  }else{
    this->b = 1;
    this->a = -(pt2.y-pt1.y) / (pt2.x-pt1.x);
    this->c = -(pt1.y + a*pt1.x);
  }
}

Line2d::Line2d(Point2d v): v(v), vec(v){
  this->u = Point2d(0,0);
  // Check duplicate point
  if(v.x == 0 && v.y == 0) throw std::invalid_argument(
        "Identical points used to create Line2d.");

  computeABC(Point2d(0,0), v);
}

Line2d::Line2d(Point2d u, Point2d v): u(u), v(v){
  this->vec = v - u;
  if(v == u) throw std::invalid_argument(
        "Identical points used to create Line2d.");

  computeABC(u, v);
}

double Line2d::length() const{
  return sqrt(pow(v.x-u.x,2) + pow(v.y-u.y,2));
}

Line2d Line2d::normalize(){
  auto len = this->length();
  return Line2d({this->vec.x/len, this->vec.y/len});
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

double Line2d::getA() const{return a;}
double Line2d::getB() const{return b;}
double Line2d::getC() const{return c;}

} //namespace cg