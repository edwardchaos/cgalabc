#include "Light.h"

#include "type.h"

namespace cg{

PointLight::PointLight(const Vector4d &position):
position(position){}

DirectionLight::DirectionLight(Vector3d in_direction){
  if(in_direction.norm() > -cg::EPS && in_direction.norm() < cg::EPS){
    throw std::invalid_argument("Directional light vector cannot be length 0");
  }
  in_direction.normalize();
  this->direction = in_direction;
}

} // namespace cg
