#pragma once

#include "olcPixelGameEngine.h"

#include "Mesh.h"
#include "Camera.h"
#include "Light.h"
#include "type.h"

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
   *
   * args:
   * cam: The camera to view with
   * mesh: The mesh to draw
   * lights: lights in the scene
   * shading_method: cg::FLAT, cg::BLINN_PHONG
   */
  void draw(const Camera_ptr &cam, const Mesh_ptr &mesh,
            const std::unordered_map<std::string, Light_ptr> &lights,
            Shade shading_method=FLAT);

  void shadeAndDrawTriangle(
      const Triangle& triangle,
      const std::unordered_map<std::string, Light_ptr> &lights);

  /*
   * Main shading operations.
   *
   * All Vector3d arguments associate to their r,g,b counterpart
   * eg material_ambience[0]: red component of material ambience
   * material_ambience[1]: green component
   * material_ambience[2]: blue component
   *
   * Args:
   * material_ambience: ambience reflectivity of material [0,1]
   * material_diffuse: diffusion reflectivity of material [0,1]
   * In the absence of external texture, material diffuse value mainly
   * discerns color
   *
   * material_specular: specular reflectivity of material [0,1]
   * surface_normal: surface normal unit vector
   * point_to_light: unit vector direction from surface point to light source
   * cam_2_light_halfway: slerp(point_to_cam, point_to_light, 0.5);
   * glossiness_exponent: affects illusion of glossiness [0,1000]
   * material_emittance: emission of color in absence of light source
   * light_ambience: ambience reflection of light source [0,1] (fancy light?)
   * light_diffuse: diffusion reflection of light source [0,1]
   * light_specular: specular reflection of light source [0,1]
   * texture: color from texture, Vector3d(1,1,1) if no texture
   *
   * equation:
   * I=(m_amb*l_amb, m_dif*l_dif(n.dot(l)) + m_emit)*texture_color
   *  +m_spec*l_spec(n.dot(halfway_cam_to_light))
   *
   *  Note: blinn-phong shading (halfway dir between camera and light)
   */
  Vector3d shade(const Vector3d &material_ambience,
                 const Vector3d &material_diffuse,
                 const Vector3d &material_specular,
                 const Vector3d &surface_normal,
                 const Vector3d &point_to_light,
                 const Vector3d &cam_2_light_halfway,
                 double glossiness_exponent,
                 const Vector3d &material_emittance,
                 const Vector3d &light_ambience,
                 const Vector3d &light_diffuse,
                 const Vector3d &light_specular,
                 const Vector3d &color_from_texture);

  /*
   * Convenience shade function that calls the main shade function
   * See the other shade function for description
   */
  Vector3d shade(const Material_ptr &material,
                 const Light_ptr &light_source,
                 const Vector3d &point_normal,
                 const Vector3d &point_to_light_vector,
                 const Vector3d &halfway_vec,
                 const Vector3d &color_from_texture=Vector3d(1,1,1));

 private:
  std::vector<std::vector<double>> depth_buffer;
  Shade shading_method_=FLAT;
};

} //namespace cg