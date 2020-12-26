#include <gtest/gtest.h>

#include "Camera.h"

#ifdef SUCCESS
#undef SUCCESS
#endif
#include "Eigen/Dense"

using Eigen::Vector4d;
using Eigen::Vector2d;
TEST(Camera, project){
  // Test projection of known points
  double vertical_fov = 75.0*M_PI/180.0;
  double screen_width = 640;
  double screen_height = 480;
  cg::Camera cam(3.0/4.0, vertical_fov, 1, 100, screen_width, screen_height);

  // Create 3D points in homogenous coordinates
  Vector3d A(0, 15.0*tan(vertical_fov/2.0), -15);
  Vector3d B(0, 50*tan(vertical_fov/2.0), -50);
  Vector3d C(0, -B(1), -50);
  Vector3d D(0, -A(1), -15);
  Vector3d E(0, 0, -15);
  Vector3d F(0, 0, -50);

  auto a_proj = cam.projectPointInWorld(A);
  ASSERT_TRUE(a_proj.isApprox(Vector2d(0,-1)));

  auto b_proj = cam.projectPointInWorld(B);
  ASSERT_TRUE(b_proj.isApprox(Vector2d(0,-1)));
  ASSERT_TRUE(a_proj.isApprox(b_proj));

  auto c_proj = cam.projectPointInWorld(C);
  ASSERT_TRUE(c_proj.isApprox(Vector2d(0,1)));

  auto d_proj = cam.projectPointInWorld(D);
  ASSERT_TRUE(d_proj.isApprox(Vector2d(0,1)));
  ASSERT_TRUE(d_proj.isApprox(c_proj));

  auto e_proj = cam.projectPointInWorld(E);
  ASSERT_TRUE(e_proj.isApprox(Vector2d(0,0)));

  auto f_proj = cam.projectPointInWorld(F);
  ASSERT_TRUE(f_proj.isApprox(Vector2d(0,0)));
  ASSERT_TRUE(f_proj.isApprox(e_proj));
}

TEST(Camera, move){
  double vertical_fov = 75.0*M_PI/180.0;
  cg::Camera cam(3.0/4.0, vertical_fov, 1, 100, 640, 480);

  Eigen::Matrix4d forward_mat = Eigen::Matrix4d::Identity();
  forward_mat(0,0) = -1;
  forward_mat(2,3) = -9.99;
  // Move forward,
  cam.moveForward(9.99);
  ASSERT_EQ(cam.pose_world.matrix(), forward_mat);

  cam.moveForward(-3);
  // Move backward
  forward_mat(2,3) += 3;
  ASSERT_EQ(cam.pose_world.matrix(), forward_mat);

  // Strafe right, left
  cam.strafeRight(12.3);
  forward_mat(0,3) -= 12.3;
  ASSERT_EQ(cam.pose_world.matrix(), forward_mat);
  cam.strafeRight(-12.3);
  forward_mat(0,3) += 12.3;
  ASSERT_EQ(cam.pose_world.matrix(), forward_mat);

  // Yaw right, left
  cam.yawRight(M_PI*2);
  auto zeromat = cam.pose_world.matrix()-forward_mat;
  for(int i = 0; i < 4; ++i){
    for(int j = 0; j < 4; ++j){
      ASSERT_NEAR(zeromat(i,j), 0, cg::EPS);
    }
  }

  cam.yawRight(-M_PI*2);
  auto zeromat2 = cam.pose_world.matrix()-forward_mat;
  for(int i = 0; i < 4; ++i){
    for(int j = 0; j < 4; ++j){
      ASSERT_NEAR(zeromat2(i,j), 0, cg::EPS);
    }
  }
}

TEST(Camera, isFacing){
  double vertical_fov = 75.0*M_PI/180.0;
  cg::Camera cam(3.0/4.0, vertical_fov, 1, 100, 640, 480);

  // Camera's default pose is origin, Looking along the -z axis.
  cg::Triangle not_facing{
    Vector3d(0,10,-10),Vector3d(10,20,-20),Vector3d(-10,20,-20)};
  cg::Triangle not_facing2{
      Vector3d(0,10,-10),Vector3d(-10,19.9,-20),Vector3d(10,19.9,-20)};
  cg::Triangle not_facing3{
      Vector3d(0,10,-10),Vector3d(-10,19.9999,-20),Vector3d(10,19.9999,-20)};
  cg::Triangle facing{
      Vector3d(0,10,-10),Vector3d(-10,21,-20),Vector3d(10,21,-20)};
  cg::Triangle not_facing4{
      Vector3d(0,10,-10),Vector3d(10,21,-20),Vector3d(-10,21,-20)};

  ASSERT_TRUE(cam.isFacing(facing));
  ASSERT_FALSE(cam.isFacing(not_facing));
  ASSERT_FALSE(cam.isFacing(not_facing2));
  ASSERT_FALSE(cam.isFacing(not_facing3));
  ASSERT_FALSE(cam.isFacing(not_facing4));
}


TEST(blank, test){
 /*
  *tfTriangleWorldToCam
  * projectPointInCameraFrame
  * clipNear
  * clipScreen2D
  * tfPointWorldToCam
  * moveTo
  */
}