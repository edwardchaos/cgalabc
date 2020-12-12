#include "Polygon.h"

#include <stdexcept>
#include <sstream>
#include <unordered_set>

#include "Algo2d.h"

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

void Polygon::constructEdges(){
  // Create vector of edges for convenience of future algorithms
  for(int i = 0; i < vertices_.size()-1; ++i){
    edges_.emplace_back(vertices_[i], vertices_[i+1]);
  }
}

bool Polygon::verifyPolygon(){
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

  this->constructEdges();

  for(int i = 0; i < edges_.size()-1; ++i){
    for(int j = i+1; j < edges_.size(); ++j){
      auto &l1 = edges_[i];
      auto &l2 = edges_[j];

      // We know adjacent lines are connected so they intersect.
      // If 2 lines aren't adjacent and they intersect, it's a problem.
      if( (i!=j-1 && i==0&&j!=edges_.size()-1)
          && intersects(l1,l2)){
        throw std::invalid_argument(
            "Cannot create polygon; Non adjacent edges intersect");
      }

      // Adjacent edges can still overlap though
      if(i==j-1 && isParallel(l1,l2) && l1.dot(l2) < 0)
        throw std::invalid_argument(
            "Cannot create polygon; Adjacent edges overlap");
    }
  }

  return true;
}

std::vector<Point2d> Polygon::vertices()const{return vertices_;}

bool Polygon::isConvex() const{

  return false;
}

double Polygon::area() const{
  return 0.0;
}

bool Polygon::containsPoint(const Point2d &pt) const{
  return false;
}

} // namespace cg
