#include <gtest/gtest.h>

#include <iostream>
#include "Camera.h"

#ifdef SUCCESS
#undef SUCCESS
#endif
#include "Eigen/Dense"

using Eigen::Matrix4d;
using Eigen::Vector4d;
using Eigen::Vector2d;
TEST(Camera, project_point_in_world){
  // Test projection of known points
  double vertical_fov = 75.0*M_PI/180.0;
  double screen_width = 640;
  double screen_height = 480;
  cg::Camera cam(3.0/4.0, vertical_fov, 1, 100, screen_width, screen_height);

  // Create 3D points in world frame
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

TEST(Camera, local_move){
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

TEST(Camera, global_move){
  Vector4d position_in_world(-1,1,-1,1);
  Vector4d look_dir(1,0,0,0);
  Vector4d up(0,1,0,0);

  double vertical_fov = 75.0*M_PI/180.0;
  double screen_width = 640;
  double screen_height = 480;
  cg::Camera cam(3.0/4.0, vertical_fov, 1, 100, screen_width, screen_height);

  cam.moveTo(position_in_world, look_dir, up);

  Matrix4d pose_gt = Matrix4d::Zero();
  pose_gt.rightCols(1) = position_in_world;
  pose_gt(0,2) = -1;
  pose_gt(1,1) = 1;
  pose_gt(2,0) = -1;
  ASSERT_TRUE(cam.pose_world.matrix().isApprox(pose_gt));

  cam.moveTo(Vector4d(-1,-1,0,1),
             Vector4d(0,1,0,0),
             Vector4d(-1,0,0,0));

  Matrix4d pose_gt2 = Matrix4d::Zero();
  pose_gt2.rightCols(1) = Vector4d(-1,-1,0,1);
  pose_gt2(2,0) = -1;
  pose_gt2(0,1) = -1;
  pose_gt2(1,2) = -1;

  ASSERT_TRUE(cam.pose_world.matrix().isApprox(pose_gt2));
 }

 TEST(Camera, coordinate_transformation){
  double vertical_fov = 75.0*M_PI/180.0;
  double screen_width = 640;
  double screen_height = 480;
  cg::Camera cam1(3.0/4.0, vertical_fov, 1, 100, screen_width, screen_height);

  Matrix4d orientation_1 = Matrix4d::Zero();
  orientation_1(0,2) = -1;
  orientation_1(1,1) = 1;
  orientation_1(2,0) = -1;
  cam1.pose_world.position = Vector4d(-1,1,-1,1);
  cam1.pose_world.orientation = orientation_1;

  cg::Camera cam2(3.0/4.0, vertical_fov, 1, 100, screen_width, screen_height);

  Matrix4d orientation_2 = Matrix4d::Zero();
  orientation_2(2,0) = -1;
  orientation_2(0,1) = -1;
  orientation_2(1,2) = -1;
  cam2.pose_world.position = Vector4d(-1,-1,0,1);
  cam2.pose_world.orientation = orientation_2;

  Vector3d pt_world(-1,1,0);
  auto pt_cam1 = cam1.tfPointWorldToCam(pt_world);
  auto pt_cam2 = cam2.tfPointWorldToCam(pt_world);

  ASSERT_TRUE(pt_cam1.isApprox(Vector4d(-1,0,0,1)));
  ASSERT_TRUE(pt_cam2.isApprox(Vector4d(0,0,-2,1)));

  try{
    auto pt_cube1 = cam1.tfPointCameraToCube(pt_cam1.head<3>());
    FAIL() << "Should throw exception";
  }catch(std::exception &e){
    ASSERT_STREQ(e.what(),
                 "Cannot project a point with z = 0; Perhaps clip first by "
                 "the near plane.");
  }
  auto pt_cube2 = cam2.tfPointCameraToCube(pt_cam2.head<3>());
  ASSERT_DOUBLE_EQ(pt_cube2(0), 0);
  ASSERT_DOUBLE_EQ(pt_cube2(1), 0);
  ASSERT_TRUE(pt_cube2(2) < 1.0);
  ASSERT_TRUE(pt_cube2(2) > -1.0);

  cg::Triangle tri_world(Vector3d(-1,1,0),
                         Vector3d(0,2,0),
                         Vector3d(0,1,0));

  auto tri_cam1 = cam1.tfTriangleWorldToCam(tri_world);
  auto tri_cam2 = cam2.tfTriangleWorldToCam(tri_world);

  ASSERT_TRUE(tri_cam1.points[0].isApprox(Vector3d(-1,0,0)));
  ASSERT_TRUE(tri_cam1.points[1].isApprox(Vector3d(-1,1,-1)));
  ASSERT_TRUE(tri_cam1.points[2].isApprox(Vector3d(-1,0,-1)));

  ASSERT_TRUE(tri_cam2.points[0].isApprox(Vector3d(0,0,-2)));
  ASSERT_TRUE(tri_cam2.points[1].isApprox(Vector3d(0,-1,-3)));
  ASSERT_TRUE(tri_cam2.points[2].isApprox(Vector3d(0,-1,-2)));

  try{
    auto cube1 = cam1.tfPointWorldToCube(pt_world);
  }
  catch(std::exception &e){
    ASSERT_STREQ(e.what(),
                 "Cannot project a point with z = 0; Perhaps clip first by "
                 "the near plane.");
  }
  auto cube2 = cam2.tfPointWorldToCube(pt_world);
  ASSERT_TRUE(pt_cube2.isApprox(cube2));
 }

 TEST(Camera, clipping){
   /*
   * clipNear
   * clip2DEdge
   * clipScreen2D
   */

}