#include <gtest/gtest.h>

#include "Camera.h"

using Eigen::Vector4d;
TEST(Camera, project){
  // Test projection of known points
  double vertical_fov = 75.0*M_PI/180.0;
  cg::Camera cam(3.0/4.0, vertical_fov, 1, 100);

  // Create 3D points in homogenous coordinates
  Vector4d A(0, 15.0*tan(vertical_fov/2.0), -15, 1);
  Vector4d B(0, 50*tan(vertical_fov/2.0), -50, 1);
  Vector4d C(0, -B(1), -50, 1);
  Vector4d D(0, -A(1), -15, 1);
  Vector4d E(0, 0, -15, 1);
  Vector4d F(0, 0, -50, 1);

  auto a_proj = cam.projectPoint(A);
  ASSERT_NE(a_proj, nullptr);
  ASSERT_EQ(*a_proj, cg::Point2d(0,-1));

  auto b_proj = cam.projectPoint(B);
  ASSERT_NE(b_proj, nullptr);
  ASSERT_EQ(*b_proj, cg::Point2d(0,-1));
  ASSERT_EQ(*a_proj, *b_proj);

  auto c_proj = cam.projectPoint(C);
  ASSERT_NE(c_proj, nullptr);
  ASSERT_EQ(*c_proj, cg::Point2d(0,1));

  auto d_proj = cam.projectPoint(D);
  ASSERT_NE(d_proj, nullptr);
  ASSERT_EQ(*d_proj, cg::Point2d(0,1));
  ASSERT_EQ(*d_proj, *c_proj);

  auto e_proj = cam.projectPoint(E);
  ASSERT_NE(e_proj, nullptr);
  ASSERT_EQ(*e_proj, cg::Point2d(0,0));

  auto f_proj = cam.projectPoint(F);
  ASSERT_NE(f_proj, nullptr);
  ASSERT_EQ(*f_proj, cg::Point2d(0,0));
  ASSERT_EQ(*f_proj, *e_proj);
}