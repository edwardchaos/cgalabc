#include "Polygon.h"

namespace cg{

Polygon::Polygon(){

}

Polygon::Polygon(std::vector<Point2d> points){
  points.push_back(points[0]);
}

std::vector<Point2d> Polygon::vertices(){return vertices_;}

bool Polygon::isConvex() const{

}

double Polygon::area() const{

}

bool Polygon::containsPoint(const Point2d &pt) const{

}

} // namespace cg
