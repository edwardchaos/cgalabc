#include <gtest/gtest.h>

#include "Mesh.h"

using Eigen::Vector3d;

TEST(Mesh, create){
  cg::Mesh m;
  m.tris.emplace_back(Vector3d(0,0,0),Vector3d(2,0,0),Vector3d(1,1,0));
  m.tris.emplace_back(Vector3d(3,1,0),Vector3d(1,1,0),Vector3d(2,0,0));

  ASSERT_EQ(m.tris[0].edges()[0], Vector3d(2,0,0));
  ASSERT_EQ(m.tris[0].edges()[1], Vector3d(-1,1,0));
  ASSERT_EQ(m.tris[0].edges()[2], Vector3d(-1,-1,0));
  ASSERT_EQ(m.tris[0].face_unit_normal(), Vector3d(0,0,1));
}

TEST(Mesh, face_normal){
  cg::Triangle triangle{
      Vector3d(0,-1,-10),{-1,-1,-10},{0,1,-10},
      Vector2d(0,1),{1,1},{0,0}};

  auto norm = triangle.face_unit_normal();
  ASSERT_TRUE(norm.isApprox(Vector3d(0,0,1)));
}
