#include "Light.h"

#include "type.h"

namespace cg{

PointLight::PointLight(const Vector4d &position):
position(position){}

PointLight::PointLight(const PointLight &other){
this->position=other.position;
}

Vector3d PointLight::getDirection(const Vector3d &pt){
  Vector3d dir = position.head<3>() - pt;
  dir.normalize();

  // It's techinically possible for direction to be length 0, but shouldn't
  // be allowed normally.
  if(dir.norm() < cg::EPS && dir.norm() > -cg::EPS)
    throw std::invalid_argument("Light is directly on the surface point, "
                                "perhaps check for collision");
  return dir;
}

void PointLight::transformToCam(const Matrix4d &tf){
  position = tf*position;
}

DirectionLight::DirectionLight(Vector3d in_direction){
  if(in_direction.norm() > -cg::EPS && in_direction.norm() < cg::EPS){
    throw std::invalid_argument("Directional light vector cannot be length 0");
  }
  in_direction.normalize();
  this->direction = in_direction;
}

DirectionLight::DirectionLight(const DirectionLight &other){
  this->direction = other.direction;
}

Vector3d DirectionLight::getDirection(const Vector3d &pt){
  return direction;
}

void DirectionLight::transformToCam(const Matrix4d &tf){
  direction = tf.block(0,0,3,3)*direction;
}

} // namespace cg
