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
  Eigen::Vector3d edges[3]; // Edge vectors also ordered counter clockwise

  Triangle(Eigen::Vector3d pt1, Eigen::Vector3d pt2, Eigen::Vector3d pt3);

  /*
   * Returns the unit normal vector of the triangle's surface.
   */
  Eigen::Vector3d unit_normal() const;
};

struct Mesh{
  std::vector<Triangle> tris;
};

} // namespace cg