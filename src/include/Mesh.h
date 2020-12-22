#pragma once

#include <utility>
#include <vector>

#ifdef Success
  #undef Success
#endif
#include <Eigen/Dense>

namespace cg{

/*
 * Triangle points are in counter clockwise order. Surface normal is positive
 * in the direction by the right hand rule
 */
struct Triangle{
  Eigen::Vector3d points[3];
  Eigen::Vector3d edge[3]; // Edge vectors also ordered counter clockwise

  Triangle(Eigen::Vector3d pt1, Eigen::Vector3d pt2, Eigen::Vector3d pt3){
    points[0] = std::move(pt1);
    points[1] = std::move(pt2);
    points[2] = std::move(pt3);

    edge[0] = points[1] - points[0];
    edge[1] = points[2] - points[1];
    edge[2] = points[0] - points[2];
  }

  /*
   * Returns the unit normal vector of the triangle's surface.
   */
  Eigen::Vector3d unit_normal() const{
    auto unit_normal = edge[0].cross(edge[1]);
    unit_normal.normalize();
    return unit_normal;
  }
};

struct Mesh{
  std::vector<Triangle> tris;
};

} // namespace cg