#include "Light.h"

#include "type.h"

namespace cg{

PointLight::PointLight(const Vector4d &position):Light(),position(position){}

PointLight::PointLight(const PointLight &other):Light(other){
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

Light_ptr PointLight::clone() const{
  return std::make_shared<PointLight>(*this);
}

DirectionLight::DirectionLight(Vector3d in_direction):Light(){
  if(in_direction.norm() > -cg::EPS && in_direction.norm() < cg::EPS){
    throw std::invalid_argument("Directional light vector cannot be length 0");
  }
  in_direction.normalize();
  this->direction = in_direction;
}

DirectionLight::DirectionLight(const DirectionLight &other):Light(other){
  this->direction = other.direction;
}

Vector3d DirectionLight::getDirection(const Vector3d &pt){
  return direction;
}

void DirectionLight::transformToCam(const Matrix4d &tf){
  direction = tf.block(0,0,3,3)*direction;
}

Light_ptr DirectionLight::clone() const{
  return std::make_shared<DirectionLight>(*this);
}
} // namespace cg
