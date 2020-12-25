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

  // Translation

  // Apply transformation on point
}

TEST(Utility, plane_line_intersect){

}

TEST(Utility, line_line_intersect){

}


