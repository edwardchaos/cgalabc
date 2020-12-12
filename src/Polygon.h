#pragma once

#include <vector>
#include "type.h"

namespace cg{

/*
 * Vertices of the polygon are ordered counter clockwise (cross product is
 * positive). Last point is equal to the first point.
 */
class Polygon{
 public:
  Polygon();

  // Input is counter clockwise ordered points. If the last point is not yet
  // equal to first point (to close the polygon), the first point is appended
  // to close the polygon.
  Polygon(std::vector<Point2d> &points);
  Polygon(std::initializer_list<Point2d> points);

  bool verifyPolygon();

  // getters
  std::vector<Point2d> vertices()const;

  // True if this polygon is convex, false otherwise
  bool isConvex() const;

  /*
   * Returns area of this convex/non-convex polygon
   */
  double area() const;

  /*
   * Returns true if this polygon contains the point.
   * Including_edge: true treats points on the edge of the polygon as
   * contained in the polygon. Setting to false would return false for points
   * on the edge.
   */
  bool containsPoint(const Point2d &pt, bool including_edge) const;

 private:
  std::vector<Point2d> vertices_;
  std::vector<Line2d> edges_;

  /*
   * Given vertices of the polygon, build consecutive edges.
   * Warning: Does not check for constructing an edge with duplicate points.
   */
  void constructEdges();
};
} // namespace cg