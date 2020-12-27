#include "Mesh.h"

namespace cg{

Triangle::Triangle(
    Eigen::Vector3d pt1, Eigen::Vector3d pt2, Eigen::Vector3d pt3){
  points[0].head<3>() = pt1;
  points[0][3] = 1;
  points[1].head<3>() = pt2;
  points[1][3] = 1;
  points[2].head<3>() = pt3;
  points[2][3] = 1;
}

std::vector<Eigen::Vector3d> Triangle::edges() const{
  return {(points[1] - points[0]).head<3>(),
          (points[2] - points[1]).head<3>(),
          (points[0] - points[2]).head<3>()};
}

Eigen::Vector3d Triangle::unit_normal() const{
  auto e = edges();
  auto unit_normal = e[0].cross(e[1]);
  unit_normal.normalize();
  return unit_normal;
}

} // namespace cg


