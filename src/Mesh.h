#pragma once

#include <utility>
#include <vector>

#ifdef Success
  #undef Success
#endif
#include <Eigen/Dense>

namespace cg{

struct Triangle{
  Eigen::Vector3d points[3];
  Triangle(Eigen::Vector3d pt1, Eigen::Vector3d pt2, Eigen::Vector3d pt3){
    points[0] = std::move(pt1);
    points[1] = std::move(pt2);
    points[2] = std::move(pt3);
  }
};

struct Mesh{
  std::vector<Triangle> tris;
};

} // namespace cg