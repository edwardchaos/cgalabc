#pragma once

#include <string>
#include <sstream>
#include <fstream>

#ifdef Success
  #undef Success
#endif
#include <Eigen/Dense>

#include "Mesh.h"

using Eigen::Vector3d;
using Eigen::Vector2d;

namespace cg{

/*
 * Create mesh from obj file
 * Triangle face vertices need to be counter clockwise ordered.
 */
Mesh_ptr loadOBJ(const std::string& path_to_obj, bool ccw_points);

/*
 * Creates a wireframe cube for of unit size, can be used for debugging.
 */
Mesh_ptr cube();

/*
 * Rotate counterclockwise by theta radian along the +X axis
 */
Eigen::Matrix4d rotateX(double theta);

/*
 * Rotate counterclockwise by theta radian along the +Y axis
 */
Eigen::Matrix4d rotateY(double theta);

/*
 * Rotate counterclockwise along the +Z axis
 */
Eigen::Matrix4d rotateZ(double theta);

/*
 * returns a 4x4 matrix for a 3D translation
 */
Eigen::Matrix4d translation(double dx, double dy, double dz);

Eigen::Vector4d transformPoint(const Eigen::Vector4d &pt,
                               const Eigen::Matrix4d& tf);

/*
 * Given a point on a plane along with the unit normal vector pointing in the
 * "in" side of the plane as well as 2 points that define a line, returns the
 * point of intersection between plane and line or nullptr if they do not
 * intersect.
 */
std::shared_ptr<Vector3d> planeLineIntersect(
    const Vector3d &pt1, const Vector3d &pt2,
    const Vector3d &plane_unit_normal, const Vector3d &pt_on_plane);

/*
 * 2D version of planeLineIntersect.
 * See planeLineIntersect for the analogous explanation
 */
std::shared_ptr<Vector2d> lineLineIntersect2d(
    const Vector2d &pt1, const Vector2d &pt2,
    const Vector2d &line_unit_normal, const Vector2d &pt_on_line);

} // namespace cg
