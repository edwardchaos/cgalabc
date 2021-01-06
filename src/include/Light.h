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
  Vector3d La= Vector3d(1,1,1); // Light ambience reflectivity
  Vector3d Ld= Vector3d(1,1,1); // Light diffusion reflectivity
  Vector3d Ls= Vector3d(1,1,1); // Light specular reflectivity

  Light() = default;

  // Returns direction the light is pointing toward
  // Make negative if you need point to light direction.
  virtual Vector3d getDirection(const Vector3d &pt)=0;

  // Transforms this light by left multiplying a rigid body transformation
  // matrix.
  virtual void transform(const Matrix4d &tf)=0;

  // Returns a clone of this light. Possible use case: transforming to
  // camera frame so original light remains in same position in world frame
  virtual Light_ptr clone()const=0;
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
  void transform(const Matrix4d &tf) override;

  Light_ptr clone() const override;
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
  void transform(const Matrix4d &tf) override;

  Light_ptr clone() const override;
};

} // namespace cg