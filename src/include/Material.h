#pragma once

#include <memory>

#include <olcPixelGameEngine.h>

namespace cg{

struct Material{
  double ka[3]={1,1,1}; // material ambient reflectivity
  double kd[3]={1,1,1}; // material diffusion reflectivity
  double ks[3]={1,1,1}; // material specular reflectivity
  // Material emission (base brightness in the absence of a light source)
  double ke[3]={0.02,0.02,0.02};
  double Ns; // Glossiness exponent
  std::shared_ptr<olc::Sprite> texture = nullptr;

  Material()=default;

};

typedef std::shared_ptr<Material> Material_ptr;
} // namespace cg