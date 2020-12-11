#include "Polygon.h"

namespace cg{

Polygon::Polygon(){};

Polygon::Polygon(std::initializer_list<Point2d> ccw_verts):vertices_(ccw_verts){
  if(vertices_.back() != vertices_.front())
    vertices_.push_back(vertices_.front());

  this->verifyPolygon();
}

Polygon::Polygon(std::vector<Point2d> &ccw_verts):vertices_(ccw_verts){
  if(vertices_.back() != vertices_.front())
    vertices_.push_back(vertices_.front());

  this->verifyPolygon();
}

bool Polygon::verifyPolygon() const{
  // At least 3 unique points (minimal is triangle)

  // All edges pairwise do not overlap

  // No edges cross (apart from points where consecutive edges connect)
  throw int(4);
  return true;
}

std::vector<Point2d> Polygon::vertices(){return vertices_;}

bool Polygon::isConvex() const{

}

double Polygon::area() const{

}

bool Polygon::containsPoint(const Point2d &pt) const{

}

} // namespace cg
