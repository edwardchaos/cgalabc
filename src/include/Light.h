#pragma once

#include <memory>
#include <Eigen/Dense>

using Eigen::Vector4d;
using Eigen::Vector3d;
using Eigen::Matrix4d;

namespace cg{

struct Light;
typedef std::shared_ptr<Light> Light_ptr;

struct Light{
  Vector3d La= Vector3d(0.9,0.9,0.9); // Light ambience reflectivity
  Vector3d Ld= Vector3d(0.9,0.9,0.9); // Light diffusion reflectivity
  Vector3d Ls= Vector3d(0.9,0.9,0.9); // Light specular reflectivity

  Light() = default;

  // Return direction unit vector from point TO light source
  // stress : NOT FROM
  virtual Vector3d getDirection(const Vector3d &pt)=0;

  // Transforms this light into a target camera's coordinate frame by the
  // parameter tf
  virtual void transformToCam(const Matrix4d &tf)=0;

  virtual Light_ptr clone()=0;
};

struct PointLight : public Light{
  Vector4d position;

  PointLight() = default;
  PointLight(const Vector4d &position);
  PointLight(const PointLight &other);

  Vector3d getDirection(const Vector3d &pt) override;

  /*
   * Transforms the point light into a target camera's coordinate frame. The
   * transform argument is left multiplied with the point.
   */
  void transformToCam(const Matrix4d &tf) override;

  Light_ptr clone() override;
};

struct DirectionLight : public Light{
  Vector3d direction;

  DirectionLight() = default;
  DirectionLight(Vector3d direction);
  DirectionLight(const DirectionLight &other);

  Vector3d getDirection(const Vector3d &pt) override;

  /*
   * Transforms the direction into a target camera frame. Only the 3x3
   * rotation of the 4x4 tf input is left multiplied.
   */
  void transformToCam(const Matrix4d &tf) override;

  Light_ptr clone() override;
};

} // namespace cg