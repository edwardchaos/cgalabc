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
  Vector3d vertex_normals[3];

  Triangle() = default;
  //Triangle vertices only
  Triangle(const Vector3d& pt1, const Vector3d &pt2, const Vector3d &pt3);
  //Triangle with texture map
  Triangle(const Vector3d &pt1, const Vector3d &pt2, const Vector3d &pt3,
           const Vector2d &t1, const Vector2d &t2, const Vector2d &t3);
  // Vertices and vertex normals
  Triangle(const Vector3d &pt1, const Vector3d &pt2, const Vector3d &pt3,
           const Vector3d &norm1, const Vector3d &norm2, const Vector3d &norm3);
  // Vertices, textures, and vertex normals
  Triangle(const Vector3d &pt1, const Vector3d &pt2, const Vector3d &pt3,
           const Vector2d &t1, const Vector2d &t2, const Vector2d &t3,
           const Vector3d &norm1, const Vector3d &norm2, const Vector3d &norm3);

  /*
   * Returns the unit normal vector of the triangle's surface.
   * This may be different from the triangle's vertex normals.
   */
  [[nodiscard]] Vector3d face_unit_normal() const;

  [[nodiscard]] std::vector<Vector3d> edges() const;

  void defaultTextureMap();
  void defaultVertexNorms();

};

struct Mesh{
  std::vector<Triangle> tris;
  Pose pose;
};

/* An explicit 2D triangle class for after perspective transform is done on
 * the 3D triangle.
 */
struct Triangle2D{
 public:
  Vector3d points[3]; // x y w
  Vector3d t[3]; // x y w
  Vector3d vertex_normals[3];

  Triangle2D() = default;
  Triangle2D(const Vector2d& pt1, const Vector2d &pt2, const Vector2d &pt3);
  Triangle2D(const Vector2d &pt1, const Vector2d &pt2, const Vector2d &pt3,
             const Vector2d &t1, const Vector2d &t2, const Vector2d &t3);

  void defaultTextureMap();
  void defaultVertexNorms();

};

} // namespace cg