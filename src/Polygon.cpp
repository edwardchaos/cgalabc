#include "Polygon.h"

#include <cassert>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <unordered_set>
#include <vector>

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

bool Polygon::operator==(const Polygon& other) const{
  if(vertices_.size() != other.vertices().size()) return false;

  std::vector<Point2d> this_unique_verts(vertices_);
  std::vector<Point2d> other_unique_verts(other.vertices());

  // Remove duplicate last point from both vertex sets
  this_unique_verts.pop_back();
  other_unique_verts.pop_back();

  // Equivalent to verifying 2 arrays, where one is the rotated version of
  // the other
  int start_idx = -1;
  for(int i = 0; i < other_unique_verts.size(); ++i){
    if(other_unique_verts[i] != this_unique_verts.front()) continue;
    start_idx = i;
    break;
  }
  if(start_idx == -1) return false;

  for(int i = 0 ; i < this_unique_verts.size(); ++i){
    int other_idx = (start_idx + i) % this_unique_verts.size();
    if(this_unique_verts[i] != other_unique_verts[other_idx]) return false;
  }

  return true;
}

bool Polygon::operator!=(const Polygon& other)const{
  return !((*this)==other);
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

/*
 * Accumulate the wedge product of a vector spanning from origin to the
 * tail of an edge with the edge. Either divide by 2 each iteration or once
 * at the end. Area outside of the polygon is subtracted (because ordering
 * of vertices is counterclockwise) and area contained in the polygon is
 * added.
 */

double Polygon::area() const{
  double area = 0;
  for(const auto & edge : edges_){
    if(edge.pt1().x() == 0 && edge.pt1().y() == 0) continue;
    Line2d origin_to_u(edge.pt1());
    area += origin_to_u.cross(edge);
  }

  return area/2.0;
}

/*
 * Winding algorithm, angles should sum to 2pi if point is within the polygon.
 *
 * For each edge AB, draw line from tail A to point to create line AP.
 * If point is on left side of AB, add the angle
 * If point is on the right side of AB, subtract angle
 *
 */
bool Polygon::containsPoint(const Point2d &pt, bool including_edge) const{
  double angle_sum = 0;

  for(int i = 0; i < edges_.size(); ++i) {
    auto point_line_dist = distancePointToLineSegment(pt, edges_[i]);
    // The point is on the edge
    bool point_on_edge = point_line_dist > -EPS && point_line_dist < EPS;
    if (point_on_edge && including_edge) return true;
    else if(point_on_edge && !including_edge) return false;

    auto cur_angle = angle(edges_[i].pt1(), pt, edges_[i].pt2());
    if (edges_[i].cross(Line2d(edges_[i].pt1(), pt)) > EPS) {
      // Point is on the left side of this edge
      angle_sum += cur_angle;
    } else angle_sum -= cur_angle;
  }

  double diff = angle_sum - 2*M_PI;
  if(diff > -EPS && diff < EPS) return true; // angle is 2pi
  return false;
}

std::pair<Polygon_ptr, Polygon_ptr> Polygon::cut(const Line2d& l) const{
  if(!this->isConvex()){
    std::cerr << "Polygon cut only supports convex polygons." << std::endl;
    return {};
  }
  std::vector<Point2d> left_pts;
  std::vector<Point2d> right_pts;

  for(int i = 0; i < edges_.size(); ++i) {
    double side1;
    if(l.pt1() == edges_[i].pt1()) side1 = 0;
    else{
      Line2d v1(l.pt1(), edges_[i].pt1());
      side1 = l.cross(v1);
    }
    double side2;
    if(l.pt1() == edges_[i].pt2()) side2 = 0;
    else{
      Line2d v2(l.pt1(), edges_[i].pt2());
      side2 = l.cross(v2);
    }

    // Determine which side the tail point of this edge belongs to
    // If cutting line is collinear with this point, add it later
    if (side1 > EPS // Tail point of this edge is on the left side of line
        && (left_pts.empty()
        || left_pts.back() != edges_[i].pt1())) { // Not a duplicate point
      left_pts.push_back(edges_[i].pt1());
    }
    else if (side1 < EPS
        && (right_pts.empty()
        || right_pts.back() != edges_[i].pt1())) { // Not a duplicate
      right_pts.push_back(edges_[i].pt1());
    }

    // Add the intercept point if there is one
    if (side1 * side2 <= EPS) { // This edge intersects the line.
      // compute intersecting point
      auto intcp = intersectPoint(l, edges_[i]);
      if(!intcp){
        // Lines are coincident, since this algorithm only deals with convex
        // polygons, it means the cutting line is coincident with one of the
        // edges. The entire polygon is either on the right or left of the
        // cutting line.

        // Determine whether the polygon is on the left or right of the
        // cutting line
        for(size_t k = 0; k < vertices_.size(); ++k){
          if(distancePointToLine(vertices_[k], l) > 0){
            if(l.cross(Line2d(l.pt1(), vertices_[k])) > 0){
              // Polygon is on the left
              return {std::make_shared<Polygon>(*this),nullptr};
            }else{
              // Polygon is on the right
              return {nullptr, std::make_shared<Polygon>(*this)};
            }
          }
        }
      }

      if(left_pts.empty()
      || left_pts.back() != *intcp) left_pts.push_back(*intcp);
      if(right_pts.empty()
      || right_pts.back() != *intcp) right_pts.push_back(*intcp);
    }

    // Determine which side the tail point of this edge belongs to
    // If cutting line is collinear with this point, add it later
    if (side2 > EPS // Tail point of this edge is on the left side of line
        && (left_pts.empty()
        || left_pts.back() != edges_[i].pt2())) { // Not a duplicate point
      left_pts.push_back(edges_[i].pt2());
    } else if (side2 < EPS
        && (right_pts.empty()
        || right_pts.back() != edges_[i].pt2())) {
      right_pts.push_back(edges_[i].pt2());
    }
  }

  Polygon_ptr left_polygon;
  if(left_pts.size() >= 3)
    left_polygon = std::make_shared<Polygon>(left_pts);
  Polygon_ptr right_polygon;
  if(right_pts.size() >=3)
    right_polygon = std::make_shared<Polygon>(right_pts);

  return {left_polygon, right_polygon};
}

} // namespace cg
