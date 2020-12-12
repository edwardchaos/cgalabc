#include "Polygon.h"

#include <stdexcept>
#include <sstream>
#include <unordered_set>
#include <utility>

typedef std::pair<double,double> dd;
typedef std::vector<double> vd;

namespace cg{

Polygon::Polygon()= default;

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
  std::unordered_set<Point2d, cg::hashPoint2d> unique_verts;

  // Check that all points are unique except for first and last
  for(int i = 0; i < vertices_.size()-1; ++i){
    if(unique_verts.find(vertices_[i])!=unique_verts.end()){
      std::stringstream s;
      s << "Repeated vertices: "
      <<"("<<vertices_[i].x()<<", "<<vertices_[i].y()<<")";
      throw std::invalid_argument(s.str());
    }
    unique_verts.emplace(vertices_[i]);
  }

  // At least 3 unique points (minimal is triangle)
  if(unique_verts.size() < 3) throw std::invalid_argument(
      "Cannot create polygon; Requires at least 3 unique vertices");

  //TODO: If edges lie ontop of each other or intersect at a point, throw
  // exception.

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
