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
namespace cg{

/*
 * Create mesh from obj file
 * Triangle face vertices need to be counter clockwise ordered.
 */
Mesh_ptr loadOBJ(std::string path_to_obj);

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

Eigen::Vector3d transformPoint(const Eigen::Vector3d &pt,
                               const Eigen::Matrix4d& tf);

} // namespace cg
