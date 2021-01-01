#include "Mesh.h"

#include <utility>

namespace cg{

Triangle::Triangle(
    Eigen::Vector3d pt1, Eigen::Vector3d pt2, Eigen::Vector3d pt3){
  points[0].head<3>() = pt1;
  points[0][3] = 1;
  points[1].head<3>() = pt2;
  points[1][3] = 1;
  points[2].head<3>() = pt3;
  points[2][3] = 1;

  // default values so clipping math doesn't crash
  t[0]=Vector3d(1,0,1);
  t[1]=Vector3d(0,0,0);
  t[2]=Vector3d(0,1,1);
}

Triangle::Triangle(
    Vector3d pt1, Vector3d pt2, Vector3d pt3,
    Vector2d t1, Vector2d t2, Vector2d t3){
  points[0].head<3>() = pt1;
  points[0][3] = 1;
  points[1].head<3>() = pt2;
  points[1][3] = 1;
  points[2].head<3>() = pt3;
  points[2][3] = 1;

  t[0].head<2>() = t1;
  t[0][2] = 1;
  t[1].head<2>() = t2;
  t[1][2] = 1;
  t[2].head<2>() = t3;
  t[2][2] = 1;
}

std::vector<Eigen::Vector3d> Triangle::edges() const{
  return {(points[1] - points[0]).head<3>(),
          (points[2] - points[1]).head<3>(),
          (points[0] - points[2]).head<3>()};
}

Eigen::Vector3d Triangle::face_unit_normal() const{
  auto e = edges();
  auto unit_normal = e[0].cross(e[1]);
  unit_normal.normalize();
  return unit_normal;
}

Triangle2D::Triangle2D(
    const Vector2d &pt1, const Vector2d &pt2, const Vector2d &pt3){
  points[0].head<2>() = pt1;
  points[0][2] = 1;
  points[1].head<2>() = pt2;
  points[1][2] = 1;
  points[2].head<2>() = pt3;
  points[2][2] = 1;

  // There is no texture
  // Default values so clipping doesn't crash
  t[0] = Vector3d(0,0, 1);
  t[1] = Vector3d(1,0, 1);
  t[2] = Vector3d(0,1, 1);
}

Triangle2D::Triangle2D(
  const Vector2d &pt1, const Vector2d &pt2, const Vector2d &pt3,
  const Vector2d &t1, const Vector2d &t2, const Vector2d &t3){
  points[0].head<2>() = pt1;
  points[0][2] = 1;
  points[1].head<2>() = pt2;
  points[1][2] = 1;
  points[2].head<2>() = pt3;
  points[2][2] = 1;

  t[0].head<2>() = t1;
  t[0][2] = 1;
  t[1].head<2>() = t2;
  t[0][2] = 1;
  t[2].head<2>() = t3;
  t[0][2] = 1;
}
} // namespace cg


