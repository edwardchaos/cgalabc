#pragma once

#include <memory>
#include <Eigen/Dense>

using Eigen::Vector4d;
using Eigen::Vector3d;

namespace cg{

struct Light;
typedef std::shared_ptr<Light> Light_ptr;

struct Light{
  double La[3] = {1,1,1}; // Light ambience reflectivity
  double Ld[3] = {1,1,1}; // Light diffusion reflectivity
  double Ls[3] = {1,1,1}; // Light specular reflectivity

  Light() = default;
};

struct PointLight : public Light{
  Vector4d position;

  PointLight() = default;
  PointLight(const Vector4d &position);
};

struct DirectionLight : public Light{
  Vector3d direction;

  DirectionLight() = default;
  DirectionLight(Vector3d direction);
};

} // namespace cg