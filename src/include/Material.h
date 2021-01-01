#pragma once

#include "olcPixelGameEngine.h"

namespace cg{

struct Material{
  double ka[3]={1,1,1};
  double kd[3]={1,1,1};
  double ks[3]={1,1,1};
  double ke[3]={1,1,1};
  double Ns; // Glossiness exponent
  std::shared_ptr<olc::Sprite> texture = nullptr;

  Material()=default;

};

} // namespace cg