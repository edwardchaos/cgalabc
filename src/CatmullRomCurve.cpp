#include "include/CatmullRomCurve.h"

namespace cg{

CatmullRomCurve::CatmullRomCurve(const Point2d& p0, const Point2d& p1,
                                  const Point2d& p2, const Point2d& p3,
                                  double dt): Curve2d(),
                                  p0(p0), p1(p1), p2(p2), p3(p3), dt_(dt){

  create();
}

void CatmullRomCurve::create(){
  createVertices();
  createEdges();
}

void CatmullRomCurve::createVertices(){
  vertices_.clear();
  double t = 0;

  while(t-1 < EPS){
    double tt = t*t;
    double ttt = tt*t;

    double b0 = 0.5*(-ttt + 2*tt - t);
    double b1 = 0.5*(2 -5*tt + 3*ttt);
    double b2 = 0.5*(t + 4*tt - 3*ttt);
    double b3 = 0.5*(-tt + ttt);

    double x_new = p0.x()*b0 + p1.x()*b1 + p2.x()*b2 + p3.x()*b3;
    double y_new = p0.y()*b0 + p1.y()*b1 + p2.y()*b2 + p3.y()*b3;

    vertices_.emplace_back(x_new,y_new);
    t += dt_;
  }
}

void CatmullRomCurve::createEdges(){
  edges_.clear();

  for(int i = 0; i < vertices_.size()-1; ++i)
    edges_.emplace_back(vertices_[i], vertices_[i+1]);
}

} // namespace cg
