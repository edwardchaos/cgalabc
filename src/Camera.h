#pragma once

namespace cg{

class Camera{
  Camera(double aspect_ratio, double vertical_fov_rad, double near, double
  far);

 private:
  double ar_, vertical_fov_, near_, far_;

  // Pose in world frame

  // Projection matrix

};

} // namespace cg
