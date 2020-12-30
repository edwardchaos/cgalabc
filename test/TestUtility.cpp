#include <gtest/gtest.h>

#include <iostream>
#include <string>

#include "Utility.h"

using Eigen::Matrix4d;
using Eigen::Vector4d;
using Eigen::Vector3d;
using Eigen::Vector2d;

TEST(Utility, load_obj){
  std::string file_path = __FILE__;
  std::string dir_path = file_path.substr(0, file_path.rfind("/"));
  dir_path += "/resource/cube.obj";
  std::cout << dir_path << std::endl;

  // Load cube obj
  bool ccw_points = true;
  auto cube_mesh_fromfile = cg::loadOBJ(dir_path, ccw_points);
  auto cube_mesh = cg::cube();

  ASSERT_NE(cube_mesh_fromfile, nullptr);

  for(int i = 0; i < cube_mesh_fromfile->tris.size(); ++i){
    auto fromfile_tri = cube_mesh_fromfile->tris[i];
    auto cube_tri = cube_mesh->tris[i];

    for(int i = 0; i < 3; ++i)
      ASSERT_TRUE(fromfile_tri.points[i].isApprox(cube_tri.points[i]));
  }
}

TEST(Utility, rigid_body_motion){
  // Rotations
  auto rotx_mat = cg::rotateX(M_PI/2.0);
  auto roty_mat = cg::rotateY(-M_PI/2.0);
  auto rotz_mat = cg::rotateZ(M_PI);

  Matrix4d rotx_gt = Matrix4d::Zero();
  rotx_gt(0,0) = 1;
  rotx_gt(1,2) = -1;
  rotx_gt(2, 1) = 1;
  rotx_gt(3,3) = 1;

  Matrix4d roty_gt = Matrix4d::Zero();
  roty_gt(0,2) = -1;
  roty_gt(1,1) = 1;
  roty_gt(2, 0) = 1;
  roty_gt(3,3) = 1;

  Matrix4d rotz_gt = Matrix4d::Zero();
  rotz_gt(0,0) = -1;
  rotz_gt(1,1) = -1;
  rotz_gt(2,2) = 1;
  rotz_gt(3,3) = 1;
  ASSERT_TRUE(rotz_mat.isApprox(rotz_gt));

  // Translation
  auto trans = cg::translation(-333.4, 29, 43);
  Matrix4d trans_gt = Matrix4d::Identity();
  trans_gt(0,3) = -333.4;
  trans_gt(1,3) = 29;
  trans_gt(2,3) = 43;

  ASSERT_TRUE(trans.isApprox(trans_gt));

  // Apply transformation on vector
  Vector4d vec(1,0,0,1);
  auto result = trans*rotz_mat*roty_mat*rotx_mat*vec;
  auto result2 = trans*rotz_mat*rotx_mat*roty_mat*vec;
  Vector4d result_gt(-333.4,29,44, 1);
  Vector4d result2_gt(-333.4, 30, 43, 1);
  ASSERT_TRUE(result.isApprox(result_gt));
  ASSERT_TRUE(result2.isApprox(result2_gt));
}

TEST(Utility, plane_line_intersect){
  Vector3d pt1(1,0,0);
  Vector3d pt2(-1,0,0);
  Vector3d pt_on_plane(0,0,0);
  Vector3d plane_normal(-1,0,0);

  double t;
  auto int_pt = cg::planeLineIntersect(pt1,pt2,plane_normal,pt_on_plane, t);
  ASSERT_NE(int_pt,nullptr);
  ASSERT_TRUE(int_pt->isApprox(pt_on_plane));
  ASSERT_DOUBLE_EQ(t, 0.5);

  Vector3d pt3(-0.00001, 0, 0);
  auto int_pt2 = cg::planeLineIntersect(pt2,pt3,plane_normal,pt_on_plane,t);
  ASSERT_EQ(int_pt2, nullptr);
  auto int_pt3 = cg::planeLineIntersect(pt1,pt3,plane_normal,pt_on_plane,t);
  ASSERT_NE(int_pt3, nullptr);
  ASSERT_TRUE(int_pt3->isApprox(pt_on_plane));
  ASSERT_NEAR(t, 1.0, 0.001);

  Vector3d pt4(0,10,0);
  auto int_pt4 = cg::planeLineIntersect(pt1,pt4,plane_normal,pt_on_plane,t);
  ASSERT_NE(int_pt4, nullptr);
  ASSERT_TRUE(int_pt4->isApprox(pt4));
  ASSERT_DOUBLE_EQ(t, 1.0);
  auto int_pt5 = cg::planeLineIntersect(pt4, pt3, plane_normal,pt_on_plane,t);
  ASSERT_NE(int_pt5, nullptr);
  ASSERT_TRUE(int_pt5->isApprox(pt4));
  ASSERT_DOUBLE_EQ(t,0.0);
  auto int_pt6 = cg::planeLineIntersect(pt2,pt4,plane_normal,pt_on_plane,t);
  ASSERT_NE(int_pt6, nullptr);
  ASSERT_TRUE(int_pt6->isApprox(pt4));
  ASSERT_DOUBLE_EQ(t,1.0);

  Vector3d pt5(0,-10,0);
  auto int_pt7 = cg::planeLineIntersect(pt5,pt4,plane_normal,pt_on_plane,t);
  ASSERT_EQ(int_pt7, nullptr);

  Vector3d pt6(1,10,0);
  auto int_pt8 = cg::planeLineIntersect(pt6,pt1,plane_normal,pt_on_plane,t);
  ASSERT_EQ(int_pt8, nullptr);

  Vector3d pt7(0.00000001,-15,0);
  Vector3d pt8(-0.00000001, 15, 0);
  auto int_78 = cg::planeLineIntersect(pt7,pt8,plane_normal,pt_on_plane,t);
  ASSERT_NE(int_78, nullptr);
  ASSERT_TRUE(int_78->isApprox(pt_on_plane));
  ASSERT_DOUBLE_EQ(t,0.5);

  // Line that is effectively parallel with the plane
  Vector3d pt9(0.00000000001,-15,0);
  Vector3d pt10(-0.0000000001, 15, 0);
  auto int_910 = cg::planeLineIntersect(pt9,pt10,plane_normal,pt_on_plane,t);
  ASSERT_EQ(int_910, nullptr);

  Vector3d pt_on_plane2(0,0,-0.01);
  Vector3d plane_normal2(-1,0,0);
  // Create test case from seg fault bug
  Vector3d pt11(-0.041560198948820293,3.0811500000000001,-0.0099063945511068813);
  Vector3d pt12(-0.041377821209217291,3.0348000000000002,-0.0099794107143673788);
  auto int_1112 = cg::planeLineIntersect(
      pt11,pt12,plane_normal2,pt_on_plane2,t);

  // Technically, they're both behind the near plane.
  ASSERT_EQ(int_1112, nullptr);

}

TEST(Utility, line_line_intersect){
  Vector2d pt1(1,0);
  Vector2d pt2(-1,0);
  Vector2d pt_on_plane(0,0);
  Vector2d plane_normal(-1,0);

  double t;
  auto int_pt = cg::lineLineIntersect2d(pt1,pt2,plane_normal,pt_on_plane,t);
  ASSERT_NE(int_pt,nullptr);
  ASSERT_TRUE(int_pt->isApprox(pt_on_plane));
  ASSERT_DOUBLE_EQ(t,0.5);

  Vector2d pt3(-0.00001, 0);
  auto int_pt2 = cg::lineLineIntersect2d(pt2,pt3,plane_normal,pt_on_plane,t);
  ASSERT_EQ(int_pt2, nullptr);
  auto int_pt3 = cg::lineLineIntersect2d(pt1,pt3,plane_normal,pt_on_plane,t);
  ASSERT_NE(int_pt3, nullptr);
  ASSERT_NEAR(t, 1.0, 0.001);
  ASSERT_TRUE(int_pt3->isApprox(pt_on_plane));

  Vector2d pt4(0,10);
  auto int_pt4 = cg::lineLineIntersect2d(pt1,pt4,plane_normal,pt_on_plane,t);
  ASSERT_NE(int_pt4, nullptr);
  ASSERT_TRUE(int_pt4->isApprox(pt4));
  ASSERT_DOUBLE_EQ(t,1.0);
  auto int_pt5 = cg::lineLineIntersect2d(pt4, pt3, plane_normal,pt_on_plane,t);
  ASSERT_NE(int_pt5, nullptr);
  ASSERT_TRUE(int_pt5->isApprox(pt4));
  ASSERT_DOUBLE_EQ(t,0.0);
  auto int_pt6 = cg::lineLineIntersect2d(pt2,pt4,plane_normal,pt_on_plane,t);
  ASSERT_NE(int_pt6, nullptr);
  ASSERT_TRUE(int_pt6->isApprox(pt4));
  ASSERT_DOUBLE_EQ(t,1.0);

  Vector2d pt5(0,-10);
  auto int_pt7 = cg::lineLineIntersect2d(pt5,pt4,plane_normal,pt_on_plane,t);
  ASSERT_EQ(int_pt7, nullptr);

  Vector2d pt6(1,10);
  auto int_pt8 = cg::lineLineIntersect2d(pt6,pt1,plane_normal,pt_on_plane,t);
  ASSERT_EQ(int_pt8, nullptr);
}


