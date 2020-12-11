#include "Line2d.h"

#include <cmath>
#include <stdexcept>
#include <stdio.h>

namespace cg{

void Line2d::computeABC(const Point2d &pt1, const Point2d &pt2){
  if(fabs(pt1.y()-pt2.y()) < EPS){ // horizontal
    b_ = 1;
    a_ = 0;
    c_ = -pt1.y();

  }else if(fabs(pt1.x()-pt2.x()) < EPS){ // vertical
    b_ = 0;
    a_ = 1;
    c_ = -pt1.x();

  }else{
    b_ = 1;
    a_ = -(pt2.y()-pt1.y()) / (pt2.x()-pt1.x());
    c_ = -(pt1.y() + a_*pt1.x());
  }
}

Line2d::Line2d(Point2d v): v_(v), vec_(v){
  u_ = Point2d(0,0);
  // Check duplicate point
  if(v.x() == 0 && v.y() == 0) throw std::invalid_argument(
        "Identical points used to create Line2d.");

  computeABC(Point2d(0,0), v);
}

Line2d::Line2d(Point2d u, Point2d v): u_(u), v_(v){
  vec_ = v - u;
  if(v == u) throw std::invalid_argument(
        "Identical points used to create Line2d.");

  computeABC(u, v);
}

Line2d::Line2d(double a, double b, double c): a_(a), b_(b), c_(c){
  if(a==0 && b==0) throw std::invalid_argument("Invalid line "
                                               "parameterization: a and b "
                                               "cannot be both 0.");
  // Vertical line
  if(b==0){
    u_ = Point2d(-c/a, 0);
    v_ = Point2d(-c/a, 1);
  }else if(a == 0){
    // Horizontal line
    u_ = Point2d(0, -c/b);
    v_ = Point2d(1, -c/b);
  }else{
    u_ = Point2d(0, -c/b);
    v_ = Point2d(1, (-a-c)/b);
  }
  vec_ = v_-u_;
}

double Line2d::length() const{
  return sqrt(pow(v_.x()-u_.x(),2) + pow(v_.y()-u_.y(),2));
}

Line2d Line2d::normalize(){
  auto len = this->length();
  return Line2d({vec_.x()/len, vec_.y()/len});
}

double Line2d::cross(const Line2d &other) const{
  // 2D cross product is same as determinant
  // Geometrically, it's equal to the area of the parallelogram of the 2
  // vectors
  return vec_.x() * other.vec().y() - vec_.y() * other.vec().x();
}

double Line2d::dot(const Line2d &other) const{
  return vec_.x() * other.vec().x() + vec_.y() * other.vec().y();
}

double Line2d::a() const{return a_;}
double Line2d::b() const{return b_;}
double Line2d::c() const{return c_;}
Point2d Line2d::vec() const{return vec_;}
Point2d Line2d::pt1() const{return u_;}
Point2d Line2d::pt2() const{return v_;}

} //namespace cg