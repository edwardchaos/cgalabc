#include "Algo2d.h"

#include <cassert>
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
  auto x = l1.b()*l2.c() - l2.b()*l1.c();
  auto y = l2.a()*l1.c() - l1.a()*l2.c();
  auto w = l1.a()*l2.b() - l2.a()*l1.b();

  if(w==0) return nullptr; // Lines are parallel, they do not intersect.
  return std::make_unique<Point2d>(x/w,y/w);
}

std::unique_ptr<Line2d> orthogonalSegment(const Point2d &pt, const Line2d &l){
  double ortho_a, ortho_b, ortho_c;

  // Compute coefficients a,b, and c of the orthogonal line
  if(l.a() == 0){ // Horizontal line
    // Orthogonal line is vertical
    ortho_b = 0;
    ortho_a = 1;
    ortho_c = -pt.x();
  }else{
    // Orthogonal line is not vertical
    ortho_b = 1;
    double ortho_slope = l.b()/l.a();
    ortho_c = -pt.y() + ortho_slope*pt.x();
    ortho_a = -ortho_slope;
  }

  // Find point of intersection between 2 lines
  Line2d ortho_line(ortho_a, ortho_b, ortho_c);
  auto int_pt = intersectPoint(l,ortho_line);
  assert(int_pt);
  if(*int_pt == pt) return nullptr;

  // Compute distance between intersection point and first point
  return std::make_unique<Line2d>(*int_pt, pt);
}

double distancePointToLine(const Point2d &pt, const Line2d &l){
  auto ortho_seg = orthogonalSegment(pt, l);
  if(!ortho_seg) return 0;
  return ortho_seg->length();
}

double distancePointToLineSegment(const Point2d &pt, const Line2d &l){
  auto ortho_seg = orthogonalSegment(pt, l);
  if(!ortho_seg) return 0;

  if(intersects(*ortho_seg, l)) return ortho_seg->length();

  // Return shortest distance between the 2 points of l and pt
  return std::min(l.pt1().dist(pt), l.pt2().dist(pt));
}

} // namespace cg