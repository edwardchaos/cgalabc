#include "Mesh.h"

namespace cg{

Triangle::Triangle(
    Eigen::Vector3d pt1, Eigen::Vector3d pt2, Eigen::Vector3d pt3){
points[0] = std::move(pt1);
points[1] = std::move(pt2);
points[2] = std::move(pt3);

edges[0] = points[1] - points[0];
edges[1] = points[2] - points[1];
edges[2] = points[0] - points[2];
}

Eigen::Vector3d Triangle::unit_normal() const{
  auto unit_normal = edges[0].cross(edges[1]);
  unit_normal.normalize();
  return unit_normal;
}

} // namespace cg


