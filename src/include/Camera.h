#pragma once

#include <memory>

#ifdef Success
  #undef Success
#endif
#include <Eigen/Dense>

#include "Point2d.h"

using Eigen::Matrix4d;
using Eigen::Vector4d;

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
   * Project a 3D point in world onto the camera's image plane.
   * 3D point is in homogenous coordinates
   *
   * IMPORTANT:
   * Returned coordinate is normalized in range [-1,1].
   * To properly draw on screen, Add 1 and Multiply x by (screen width)/2
   * Multiply y by (screen height)/2
   */
  Point2d_ptr projectPoint(const Vector4d &pt_homo) const;

 private:
  // Distance is in units of the world
  double ar_, vertical_fov_, near_plane_dist_, far_plane_dist_;

  // Pose in world frame
  Matrix4d pose_world_;

  // Projection matrix
  Matrix4d projection_mat_;

  void init();
  void constructProjectionMatrix();
};

} // namespace cg
