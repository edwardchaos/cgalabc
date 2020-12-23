#pragma once

#include <utility>
#include <vector>
#include <memory>

#ifdef Success
  #undef Success
#endif
#include <Eigen/Dense>

#include "Pose.h"

namespace cg{

struct Mesh;
typedef std::shared_ptr<Mesh> Mesh_ptr;
/*
 * Triangle points are in counter clockwise order. Surface normal is positive
 * in the direction by the right hand rule
 */
struct Triangle{
  Eigen::Vector3d points[3];

  Triangle(Eigen::Vector3d pt1, Eigen::Vector3d pt2, Eigen::Vector3d pt3);

  /*
   * Returns the unit normal vector of the triangle's surface.
   */
  Eigen::Vector3d unit_normal() const;

  std::vector<Eigen::Vector3d> edges() const;
};

struct Mesh{
  std::vector<Triangle> tris;
  Pose pose_;
};

} // namespace cg