#include "Algo2d.h"

#include <cmath>
#include "type.h"

namespace cg{
bool isParallel(const Line2d& l1, const Line2d& l2){
  if(l1.b() == 0 || l2.b() == 0)
    return l2.b() == 0 && l1.b() == 0; // Both lines are vertical
  else return fabs(l1.a()/l1.b() - l2.a()/l2.b()) < EPS;
}

bool isOrthogonal(const Line2d& l1, const Line2d& l2){
  return fabs(l1.dot(l2)) < EPS;
}

bool intersects(const Line2d& l1, const Line2d& l2){
  /*
   * Line segments DO NOT intersect if both points of one line are on one
   * side of the other line.
   */
  if(l1.pt1()==l2.pt1() || l1.pt1() == l2.pt2()
  || l2.pt2()==l1.pt2() || l2.pt1() == l1.pt2()) return true;

  // Check points of l2 from perspective of l1
  auto leftright1 = l1.cross(Line2d(l1.pt1(),l2.pt1()));
  auto leftright2 = l1.cross(Line2d(l1.pt1(),l2.pt2()));
  if(leftright1 * leftright2 > EPS) return false;

  // Check points of l1 from perspective of l2
  auto leftright3 = l2.cross(Line2d(l2.pt1(),l1.pt1()));
  auto leftright4 = l2.cross(Line2d(l2.pt1(),l1.pt2()));
  if(leftright3 * leftright4 > EPS) return false;

  return true;
}

std::unique_ptr<Point2d> intersectPoint(const Line2d& l1, const Line2d& l2){
  if(!intersects(l1,l2)) return nullptr;

  auto x = l1.b()*l2.c() - l2.b()*l1.c();
  auto y = l2.a()*l1.c() - l1.a()*l2.c();
  auto w = l1.a()*l2.b() - l2.a()*l1.b();

  // Edge case?
  return std::make_unique<Point2d>(x/w,y/w);
}
};