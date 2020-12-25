#include <gtest/gtest.h>

#include <iostream>
#include <string>

#include "Utility.h"

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

  Eigen::Matrix4d rotx_gt = Eigen::Matrix4d::Zero();
  rotx_gt(0,0) = 1;
  rotx_gt(1,2) = -1;
  rotx_gt(2, 1) = 1;
  rotx_gt(3,3) = 1;

  Eigen::Matrix4d roty_gt = Eigen::Matrix4d::Zero();
  roty_gt(0,2) = -1;
  roty_gt(1,1) = 1;
  roty_gt(2, 0) = 1;
  roty_gt(3,3) = 1;

  Eigen::Matrix4d rotz_gt = Eigen::Matrix4d::Zero();
  rotz_gt(0,0) = -1;
  rotz_gt(1,1) = -1;
  rotz_gt(2,2) = 1;
  rotz_gt(3,3) = 1;
  ASSERT_TRUE(rotz_mat.isApprox(rotz_gt));

  // Translation
  auto trans = cg::translation(-333.4, 29, 43);
  Eigen::Matrix4d trans_gt = Eigen::Matrix4d::Identity();
  trans_gt(0,3) = -333.4;
  trans_gt(1,3) = 29;
  trans_gt(2,3) = 43;

  ASSERT_TRUE(trans.isApprox(trans_gt));

  // Apply transformation on vector
  Eigen::Vector4d vec(1,0,0,1);
  auto result = trans*rotz_mat*roty_mat*rotx_mat*vec;
  auto result2 = trans*rotz_mat*rotx_mat*roty_mat*vec;
  Eigen::Vector4d result_gt(-333.4,29,44, 1);
  Eigen::Vector4d result2_gt(-333.4, 30, 43, 1);
  ASSERT_TRUE(result.isApprox(result_gt));
  ASSERT_TRUE(result2.isApprox(result2_gt));
}

TEST(Utility, plane_line_intersect){

}

TEST(Utility, line_line_intersect){

}


