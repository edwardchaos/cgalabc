#pragma once

#include <memory>

#ifdef Success
  #undef Success
#endif
#include <Eigen/Dense>

#include "Point2d.h"
#include "Mesh.h"

using Eigen::Matrix4d;
using Eigen::Vector4d;
using Eigen::Vector3d;

namespace cg{
class Camera;
typedef std::shared_ptr<Camera> Camera_ptr;
/*
 * Camera frustum that can transform the 3d world to a 2d image plane by
 * perspective transform.
 */
class Camera{
 public:
  /*
   * Camera initialized with pose at the origin and no rotation.
   *
   * Aspect ratio: height/width
   * Vertical field of view in radians
   * Near plane distance in world units
   * Far plane distance in world units
   */
  Camera(double aspect_ratio, double vertical_fov_rad, double near, double
  far);

  /*
   * Project a 3D point from world onto the camera's image plane.
   * 3D point is in homogenous coordinates
   *
   * IMPORTANT:
   * Returned coordinate is normalized in range [-1,1].
   * To properly draw on screen, Add 1 and Multiply x by (screen width)/2
   * Multiply y by (screen height)/2
   */
  Point2d_ptr projectPoint(const Vector4d &pt_homo) const;

  /*
   * Indicates whether the camera is viewing the triangle from a perspective
   * where the triangle's outer surface is facing the camera.
   *
   * i.e. (triangle point - camera position) dot (triangle normal) < 0
   */
  bool isFacing(const Triangle& tri) const;

  Pose pose_world;
 private:
  // Distance is in units of the world
  double ar_, vertical_fov_, near_plane_dist_, far_plane_dist_;

  // Projection matrix
  Matrix4d projection_mat_;

  void init();
  void constructProjectionMatrix();
};

} // namespace cg
