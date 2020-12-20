#pragma once

#include <memory>

#include "type.h"

namespace cg {
class Curve2d;
typedef std::shared_ptr<Curve2d> Curve2d_ptr;

class Curve2d{
 public:
  std::vector<Point2d> vertices(){return vertices_;}
  std::vector<Line2d> edges(){return edges_;}

 protected:
  // For purposes of drawing and integration with other geometry computations
  // such as intersection, etc curves are just short straight line segments.
  std::vector<Point2d> vertices_;
  std::vector<Line2d> edges_;
};

} // namespace cg