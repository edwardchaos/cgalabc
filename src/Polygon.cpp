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
  // All adjacent edges for left turns since vertices are ordered
  // counter-clockwise

  for(int i = 0; i < edges_.size()-1; ++i)
    if(edges_[i].cross(edges_[i+1]) < 0) return false;

  return true;
}

double Polygon::area() const{
  // Accumulate the wedge product of a vector spanning from origin to the
  // tail of an edge with the edge. Either divide by 2 each iteration or once
  // at the end. Area outside of the polygon is subtracted (because ordering
  // of vertices is counterclockwise) and area contained in the polygon is
  // added.

  double area = 0;
  for(const auto & edge : edges_){
    if(edge.pt1().x() == 0 && edge.pt1().y() == 0) continue;
    Line2d origin_to_u(edge.pt1());
    area += origin_to_u.cross(edge);
  }

  return area/2.0;
}

bool Polygon::containsPoint(const Point2d &pt, bool including_edge) const{
  return false;
}

} // namespace cg
