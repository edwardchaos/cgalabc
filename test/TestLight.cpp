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

}

