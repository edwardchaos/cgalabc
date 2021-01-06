#pragma once

#include <utility>
#include <vector>
#include <memory>

#ifdef Success
  #undef Success
#endif
#include <Eigen/Dense>

#include "Pose.h"
#include "Material.h"

using Eigen::Vector4d;
using Eigen::Vector3d;
using Eigen::Vector2d;

namespace cg{

struct Mesh;
typedef std::shared_ptr<Mesh> Mesh_ptr;
struct Triangle;
typedef std::vector<Triangle> vTriangle;
/*
 * Triangle points are in counter clockwise order. Surface normal is positive
 * in the direction by the right hand rule
 */
struct Triangle{
  // Homogenous coordinates
  Vector4d points[3]; // 3D points in world/cam frame
  Vector3d points2d[3]; // corresponding 2D perspective projected points
  Vector3d t[3]; // Texture coordinates
  Vector3d vertex_normals[3];
  Material_ptr material = nullptr;

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
  void default2DPoints();

};

struct Mesh{
  vTriangle tris;
  Pose pose;
  std::string name;
};
} // namespace cg