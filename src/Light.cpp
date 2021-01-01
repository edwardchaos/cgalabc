#include "Light.h"

namespace cg{

PointLight::PointLight(const Vector4d &position):
position(position){}

DirectionLight::DirectionLight(const Vector3d &direction):
direction(direction){}

} // namespace cg
