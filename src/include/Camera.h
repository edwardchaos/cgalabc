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
  Camera(double aspect_ratio, double vertical_fov_rad, double near,
         double far, int screen_width_px, int screen_height_px);

  /*
   * Project a 3D point in world coordinate onto the camera's image plane.
   *
   * IMPORTANT:
   * Returned coordinate is normalized in range [-1,1].
   * To properly draw on screen, Add 1 and Multiply x by (screen width)/2
   * Multiply y by (screen height)/2
   */
  //TODO: Rename to projectPointInWorld and internally call
  // projectPointInCameraFrame.
  [[nodiscard]] Point2d_ptr projectPoint(const Vector3d &pt_world) const;

  /*
   * Project a 3D point in camera coordinate onto the camera's image plane.
   */
  [[nodiscard]] Vector3d projectPointInCameraFrame(
      const Vector3d &pt_cam)const;

  /*
   * Transform a point in world coordinate to the camera's unit cube space by
   * the perspective transform.
   */
  [[nodiscard]] Vector3d transformPoint(const Vector3d &pt_world) const;

  /*
   * Transform a triangle from world coordinate to camera coordainte.
   */
  Triangle tfTriangleWorldToCam(const Triangle& tri_world) const;

  /*
   * Clip triangle in camera coordainte space by the near plane.
   * Returns 0,1, or 2 triangles depending on original triangle intersection
   * with near plane.
   */
  [[nodiscard]] std::vector<Triangle> clipNear(const Triangle& tri_cam) const;

  /*
   * Indicates whether the camera is viewing the triangle from a perspective
   * where the triangle's surface normal is facing the camera.
   *
   * Triangle is in world coordinate system.
   */
  [[nodiscard]] bool isFacing(const Triangle& tri_world) const;

  void moveTo(Vector4d position_world,
              Vector4d look_dir,
              Vector4d up);

  void moveForward(double units);
  void strafeRight(double units);
  // Rotate camera to right by theta in radians
  void yawRight(double theta_radians);

  Pose pose_world;
 private:
  // Distance is in units of the world
  double ar_, vertical_fov_, near_plane_dist_, far_plane_dist_;
  int screen_width_, screen_height_;

  // Projection matrix
  Matrix4d projection_mat_;

  void init();
  void constructProjectionMatrix();

  /*
   * Transform point in world coordinate frame to a homogenous point in
   * camera coordinate frame.
   */
  Vector4d tfPointWorldToCam(const Vector3d &pt_world) const;
};

} // namespace cg
