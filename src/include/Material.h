#pragma once

#include <memory>

#include <olcPixelGameEngine.h>

#ifdef Success
#undef Success
#endif
#include <Eigen/Dense>

using Eigen::Vector3d;
namespace cg{

struct Material{
  Vector3d ka=Vector3d(0.1,0.1,0.1); // material ambient reflectivity
  Vector3d kd=Vector3d(0.5,0.5,0.5); // material diffusion reflectivity
  Vector3d ks=Vector3d(0.2,0.2,0.2); // material specular reflectivity
  // Material emission (base brightness in the absence of a light source)
  Vector3d ke=Vector3d(0.02,0.02,0.02);
  double Ns = 1; // Glossiness exponent
  std::shared_ptr<olc::Sprite> texture = nullptr;

  Material()=default;

};

typedef std::shared_ptr<Material> Material_ptr;
} // namespace cg