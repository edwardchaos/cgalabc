#include "Camera.h"

namespace cg {


Camera::Camera(double aspect_ratio, double vertical_fov_rad, double near,
               double far): ar_(aspect_ratio), vertical_fov_
               (vertical_fov_rad), near_(near), far_(far){

}


} // namespace cg
