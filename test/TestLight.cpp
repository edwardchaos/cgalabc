#include <gtest/gtest.h>

#include <vector>
#include <memory>

#include "Light.h"

TEST(Light, getDirection){
  std::vector<cg::Light_ptr> lights;
  // Directional light pointed in the 1,1,1 direction
  lights.push_back(std::make_shared<cg::DirectionLight>(Vector3d(1,1,1)));
  // Point light positioned at 1,1,1
  lights.push_back(std::make_shared<cg::PointLight>(Vector4d(1,1,1,1)));

  // Directions of light pointed at (0,0,0)
  auto directional = lights[0]->getDirection(Vector3d(0,0,0));
  auto point_dir = lights[1]->getDirection(Vector3d(0,0,0));

  ASSERT_TRUE(directional.isApprox(Vector3d(1,1,1).normalized()));
  ASSERT_TRUE(point_dir.isApprox(Vector3d(-1,-1,-1).normalized()));

  auto directional2 = lights[0]->getDirection(Vector3d(0,1,1));
  auto point_dir2 = lights[1]->getDirection(Vector3d(0,1,1));
  ASSERT_TRUE(directional2.isApprox(Vector3d(1,1,1).normalized()));
  ASSERT_TRUE(point_dir2.isApprox(Vector3d(-1,0,0).normalized()));
}

TEST(Light, clone_transform){
  std::vector<cg::Light_ptr> lights;
  // Directional light pointed in the 1,1,1 direction
  lights.push_back(std::make_shared<cg::DirectionLight>(Vector3d(1,1,1)));
  // Point light positioned at 1,1,1
  lights.push_back(std::make_shared<cg::PointLight>(Vector4d(1,1,0,1)));

  auto cloned_dir_light = lights[0]->clone();
  auto cloned_pt_light = lights[1]->clone();

  Matrix4d tf = Matrix4d::Identity();
  // Rotation in y axis by 180 deg
  tf(0,0) = -1;
  tf(2,2) = -1;
  // Translation in z axis by 5 units
  tf(2,3) = 5;

  cloned_dir_light->transform(tf);
  cloned_pt_light->transform(tf);

  // Original light directions
  auto directional_dir = lights[0]->getDirection(Vector3d(0,0,0));
  auto point_dir = lights[1]->getDirection(Vector3d(0,0,0));
  // Cloned, transformed light directions
  auto cloned_directional_dir = cloned_dir_light->getDirection(Vector3d(0,0,0));
  auto cloned_pt_dir = cloned_pt_light->getDirection(Vector3d(0,0,0));

  // Original light is unaffected by transformation
  ASSERT_TRUE(directional_dir.isApprox(Vector3d(1,1,1).normalized()));
  ASSERT_TRUE(point_dir.isApprox(Vector3d(-1,-1,0).normalized()));

  // Cloned lights are correctly transformed
  ASSERT_TRUE(cloned_directional_dir.isApprox(Vector3d(-1,1,-1).normalized()));
  ASSERT_TRUE(cloned_pt_dir.isApprox(Vector3d(1,-1,-5).normalized()));
}
