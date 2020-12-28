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
using Eigen::Vector2d;

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
  /*
   * Triangle in world without texture
   */
  Triangle(Vector3d pt1, Vector3d pt2, Vector3d pt3);

  /*
   * Triangle in world with corresponding 2d points on a texture
   */
  Triangle(Vector3d pt1, Vector3d pt2, Vector3d pt3,
           Vector2d t1, Vector2d t2, Vector2d t3);

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

struct Triangle2D{
 public:
  Vector3d points[3]; // x y w
  Vector3d t[3]; // x y w

  Triangle2D() = default;
  Triangle2D(const Vector2d& pt1, const Vector2d &pt2, const Vector2d &pt3);
  Triangle2D(const Vector2d &pt1, const Vector2d &pt2, const Vector2d &pt3,
             const Vector2d &t1, const Vector2d &t2, const Vector2d &t3);

 private:

};

} // namespace cg