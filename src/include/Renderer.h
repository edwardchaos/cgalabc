#pragma once

#include "olcPixelGameEngine.h"

#include "Mesh.h"
#include "Camera.h"
#include "Light.h"

namespace cg{

// Inherit from PGEX so drawing functions can be called from here
class Renderer : public olc::PGEX{
 public:
  explicit Renderer(olc::PixelGameEngine* pge);

  /*
   * Clear screen
   */
  void clear();

  /*
   * The main rendering pipeline.
   * 1. Moves mesh to its pose in world. Mesh pose stored in mesh.pose
   * 2. Backface culling
   * 3. Clipping
   * 4. Perspective transform to camera view
   * 5. Scan conversion
   * 6. Draws pixels in pge window
   *
   * More:
   * Renders the image viewed of a mesh from a camera
   * Depth buffer is only cleared with this->clear(), so repeated calls
   * with multiple meshes produce occlusion
   */
  void draw(const Camera_ptr &cam, const Mesh_ptr &mesh,
            const std::unordered_map<std::string, Light_ptr> &lights);

  void shadeAndDrawTriangle(
      const Triangle& triangle,
      const std::unordered_map<std::string, Light_ptr> &lights);

 private:
  std::vector<std::vector<double>> depth_buffer;
};

} //namespace cg