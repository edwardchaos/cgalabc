#pragma once

#include <utility>
#include <vector>
#include <memory>

#ifdef Success
  #undef Success
#endif
#include <Eigen/Dense>

#include "Pose.h"

using Eigen::Vector4d;
using Eigen::Vector3d;
namespace cg{

struct Mesh;
typedef std::shared_ptr<Mesh> Mesh_ptr;
/*
 * Triangle points are in counter clockwise order. Surface normal is positive
 * in the direction by the right hand rule
 */
struct Triangle{
  // Homogenous coordinates
  Vector4d points[3]; // Coordinates in world frame
  Vector3d t[3]; // Texture coordinates

  Triangle() = default;
  Triangle(Vector3d pt1, Vector3d pt2, Vector3d pt3);

  /*
   * Returns the unit normal vector of the triangle's surface.
   */
  [[nodiscard]] Vector3d unit_normal() const;

  [[nodiscard]] std::vector<Vector3d> edges() const;
};

struct Mesh{
  std::vector<Triangle> tris;
  Pose pose;
};

} // namespace cg