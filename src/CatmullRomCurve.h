#pragma once

#include "type.h"
#include "Curve2d.h"

namespace cg{

class CatmullRomCurve : public Curve2d{
 public:
  /*
   * Catmull-Rom cubic curve from p1 to p2. p0 and p3 influence the curvature of
   * the line.
   *
   * This is similar to the bezier curve because they both have 2 anchors
   * where the curve passes through. They are both cubics.
   *
   * Unlike the bezier curve where control points can be arbitrarily selected
   * to affect curvature,
   * the catmull-rom curve can be used on a sequence of points to generate a
   * smooth curve that passes directly through all points.
   *
   * Eg. Say we have a list of points [p0, p1, p2, p3, p4, p5]
   * We can join a sequence of catmull rom curves that pass through all given
   * points (all but the first and last)
   *
   * First curve: catmull(p0,p1,p2,p3) -> curve in range [p1,p2]
   * Second curve: catmull(p1,p2,p3,p4) -> curve in range [p2,p3]
   * Third curve: catmull(p2,p3,p4,p5) -> curve in range [p3,p4]
   */
  CatmullRomCurve(const Point2d& p0, const Point2d& p1, const Point2d& p2,
                  const Point2d& p3, double dt=0.01);

 private:
  // From base class
  //  std::vector<Point2d> vertices_;
  //  std::vector<Line2d> edges_;
  Point2d p0, p1, p2, p3;
  double dt_;

  void create();
  void createVertices() override;
  void createEdges() override;
};

}