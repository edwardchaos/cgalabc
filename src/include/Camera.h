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
using Eigen::Vector2d;

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
   * Clip a triangle in image space on the 4 screen boundaries given by a
   * vector of 3 points.
   *
   * Depending on the clip results with left,top,right, and bottom edge, this
   * function may return 0 or more triangles.
   */
  [[nodiscard]] std::vector<std::vector<Vector2d>>
  clipScreen2D(const std::vector<Vector2d> &tri_img) const;

  /*
   * Clip triangles on a single 2d edge.
   * edge_unit_normal is orthonormal pointing in the 'in' screen direction.
   * pt_on_edge is a point on the edge.
   * tris is a vector of triangles represented as a vector of 2d points.
   */
  [[nodiscard]] std::vector<std::vector<Vector2d>>
  clip2DEdge(const Vector2d &edge_unit_normal,
      const Vector2d &pt_on_edge,
      const std::vector<std::vector<Vector2d>> &tris)const;

  /*
   * Indicates whether the camera is viewing the triangle from a perspective
   * where the triangle's surface normal is facing the camera.
   *
   * Triangle is in world coordinate system.
   */
  [[nodiscard]] bool isFacing(const Triangle& tri_world) const;

  /*
   * Project triangle in world frame to screen space. This is the main
   * function performing everything including:
   *
   * 1. Transformation to camera frame
   * 2. Backface culling
   * 3. Near plane clipping
   * 4. Projection to screen space
   * 5. Screen clipping (top, bottom, left, right screen edges)
   *
   * Returns a vector of triangles in screen space represented as a vector of
   * 2d points. Does not perform depth buffering.
   */
  [[nodiscard]] std::vector<std::vector<Vector2d>>
  projectTriangleFromWorld(const Triangle& tri_world) const;

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
