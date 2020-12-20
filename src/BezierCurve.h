#pragma once

#include "Curve2d.h"
#include "type.h"

namespace cg{

class BezierCurve : public Curve2d{
 public:
  /*
   * Cubic bezier curve given 2 anchors and 2 control points.
   *
   * Curve is constructed with the assumed order:
   * anchor1 -> control1 -> control2 -> anchor2
   */
  BezierCurve(const Point2d& anchor1, const Point2d& anchor2,
              const Point2d& control1, const Point2d& control2,
              double dt);
 private:
  // From base class
  //  std::vector<Point2d> vertices_;
  //  std::vector<Line2d> edges_;
  Point2d anchor1,anchor2, control1, control2;
  double dt_;

  void createVertices();
  void createEdges();
};

} // namespace cg