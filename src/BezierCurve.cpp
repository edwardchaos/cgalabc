#include "BezierCurve.h"

#include <iostream>
#include "Algo2d.h"

namespace cg{

BezierCurve::BezierCurve(const Point2d& anchor1, const Point2d& anchor2,
                         const Point2d& control1, const Point2d& control2,
                         double dt): Curve2d(),
                         dt_(dt), anchor1(anchor1), anchor2(anchor2),
                         control1(control1), control2(control2){

  create();
}

void BezierCurve::create(){
  createVertices();
  createEdges();
}

void BezierCurve::createVertices(){
  vertices_.clear();
  double t = 0;
  while(t - 1 < EPS){
    // J0, J1, J2
    auto J0 = lerp(anchor1, control1, t);
    auto J1 = lerp(control1, control2, t);
    auto J2 = lerp(control2, anchor2, t);

    // K1, K2
    auto K1 = lerp(J0, J1, t);
    auto K2 = lerp(J1, J2, t);

    // T
    auto T = lerp(K1, K2, t);
    std::cout << vertices_.size()-1 << ": ";

    vertices_.push_back(std::move(T));
    t += dt_;
  }
}

void BezierCurve::createEdges() {
  edges_.clear();

  for(int i = 0; i < vertices_.size()-1; ++i)
    edges_.emplace_back(vertices_[i], vertices_[i+1]);
}

} // namespace cg
