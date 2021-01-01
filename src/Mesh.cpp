#include "Mesh.h"

#include <utility>

namespace cg{

Triangle::Triangle(
    const Vector3d &pt1, const Vector3d &pt2, const Vector3d &pt3){
  points[0].head<3>() = pt1;
  points[0][3] = 1;
  points[1].head<3>() = pt2;
  points[1][3] = 1;
  points[2].head<3>() = pt3;
  points[2][3] = 1;

  defaultTextureMap();
  defaultVertexNorms();
}

Triangle::Triangle(
    const Vector3d &pt1, const Vector3d &pt2, const Vector3d &pt3,
    const Vector2d &t1, const Vector2d &t2, const Vector2d &t3){
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

  defaultVertexNorms();
}

Triangle::Triangle(
    const Vector3d &pt1, const Vector3d &pt2, const Vector3d&pt3,
    const Vector3d &norm1, const Vector3d &norm2, const Vector3d &norm3){
  points[0].head<3>() = pt1;
  points[0][3] = 1;
  points[1].head<3>() = pt2;
  points[1][3] = 1;
  points[2].head<3>() = pt3;
  points[2][3] = 1;

  vertex_normals[0] = norm1;
  vertex_normals[1] = norm2;
  vertex_normals[2] = norm3;

  defaultTextureMap();
}

Triangle::Triangle(
    const Vector3d &pt1, const Vector3d &pt2, const Vector3d&pt3,
    const Vector2d &t1, const Vector2d &t2, const Vector2d &t3,
    const Vector3d &norm1, const Vector3d &norm2, const Vector3d &norm3){
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

  vertex_normals[0] = norm1;
  vertex_normals[1] = norm2;
  vertex_normals[2] = norm3;
}

void Triangle::defaultTextureMap(){
  // default values to avoid unknown unknowns
  this->t[0]=Vector3d(1,0,1);
  this->t[1]=Vector3d(0,0,1);
  this->t[2]=Vector3d(0,1,1);
}

void Triangle::defaultVertexNorms(){
  auto face_norm = face_unit_normal();
  this->vertex_normals[0] = face_norm;
  this->vertex_normals[1] = face_norm;
  this->vertex_normals[2] = face_norm;
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

  defaultTextureMap();
  defaultVertexNorms();
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

  defaultVertexNorms();
}

void Triangle2D::defaultTextureMap(){
  // default values to avoid unknown unknowns
  this->t[0]=Vector3d(1,0,1);
  this->t[1]=Vector3d(0,0,1);
  this->t[2]=Vector3d(0,1,1);
}

void Triangle2D::defaultVertexNorms(){
  this->vertex_normals[0] = Vector3d(1,0,0);
  this->vertex_normals[1] = Vector3d(1,0,0);
  this->vertex_normals[2] = Vector3d(1,0,0);
}
} // namespace cg


