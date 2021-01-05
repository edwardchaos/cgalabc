#include "Renderer.h"

#include "Utility.h"

namespace cg{

Renderer::Renderer(olc::PixelGameEngine* p_pge){
  // A pointer to the main pge object so we can draw in this class
  pge = p_pge;

  // ye ol depth buffer
  depth_buffer.resize(pge->ScreenHeight()+1);
  for(auto &row:depth_buffer)
    row.assign(pge->ScreenWidth()+1, -1e9);
}

void Renderer::clear(){
  pge->Clear(olc::DARK_GREY);

  // reset depth buffer
  for(auto &row:depth_buffer)
    row.assign(pge->ScreenWidth()+1, -1e9);
}

void Renderer::draw(const Camera_ptr &cam, const Mesh_ptr &mesh,
                    const std::unordered_map<std::string, Light_ptr>&lights_world){
  for(const auto& tri : mesh->tris){
    // Mesh points are kept as they are when they're first loaded.
    // Transformations on meshes affect their pose
    // Move original mesh to its pose in world
    auto tf = mesh->pose.matrix();
    cg::Triangle tri_world = cg::transformTriangle(tri, tf);

    auto triangles_to_draw = cam->projectTriangleInWorld(tri_world);

    // Shading is performed with all entities in the camera frame. Transform
    // the lights into the camera frame.
    std::unordered_map<std::string, Light_ptr> lights_copy;
    for(const auto &light : lights_world){
      Light_ptr newlight_ptr = light.second->clone();
      // Transform light to camera frame for shading
      newlight_ptr->transform(cam->pose_world.matrix().inverse());
      lights_copy.emplace(light.first,newlight_ptr);
    }

    /* At this point, a triangle has the following member variables:
     * points: 3d point in cam frame
     * point2d: 2d point in screen space
     * t: perspective corrected texture map
     * vertex_normals: direction unit vector in cam frame
     * material: same as original
     */
    for(const auto& tri_to_draw: triangles_to_draw){
      shadeAndDrawTriangle(tri_to_draw, lights_copy);
//      pge->DrawTriangle(to_draw.points2d[0].x(),to_draw.points2d[0].y(),
//                        to_draw.points2d[1].x(),to_draw.points2d[1].y(),
//                        to_draw.points2d[2].x(),to_draw.points2d[2].y());
    }
  }
}

void Renderer::shadeAndDrawTriangle(
    const Triangle& tri,
    const std::unordered_map<std::string, Light_ptr> &lights) {
  auto pt1 = tri.points2d[0];
  auto pt2 = tri.points2d[1];
  auto pt3 = tri.points2d[2];

  // Discretize to pixels
  int p1x = std::round(pt1.x()); int p1y = std::round(pt1.y());
  int p2x = std::round(pt2.x()); int p2y = std::round(pt2.y());
  int p3x = std::round(pt3.x()); int p3y = std::round(pt3.y());

  auto tx1 = tri.t[0]; auto tx2 = tri.t[1]; auto tx3 = tri.t[2];

  Vector3d pt1_3d = tri.points[0].head<3>();
  Vector3d pt2_3d = tri.points[1].head<3>();
  Vector3d pt3_3d = tri.points[2].head<3>();
  auto n1 = tri.vertex_normals[0];
  auto n2 = tri.vertex_normals[1];
  auto n3 = tri.vertex_normals[2];

  // Sort 2D triangle vertices from top to bottom pt1 top, pt2 mid, pt3 bottom
  if(p2y < p1y){
    std::swap(p2x,p1x);std::swap(p2y,p1y);std::swap(tx2,tx1);
    std::swap(n2,n1);std::swap(pt2_3d,pt1_3d);}
  if(p3y < p1y){
    std::swap(p3x,p1x);std::swap(p3y,p1y);std::swap(tx3,tx1);
    std::swap(n3,n1);std::swap(pt3_3d,pt1_3d);}
  if(p3y < p2y){
    std::swap(p3x,p2x);std::swap(p3y,p2y);std::swap(tx3,tx2);
    std::swap(n3,n2);std::swap(pt3_3d,pt2_3d);}

  int y12 = p2y-p1y; assert(y12>=0);
  int x12 = p2x-p1x;
  int y13 = p3y-p1y; assert(y13>=0);
  int x13 = p3x-p1x;
  int y23 = p3y-p2y; assert(y23>=0);
  double dx12 = 0;
  if(y12!=0)
    dx12=(double)x12/(double)y12;
  double dx13 = 0;
  if(y13!=0)
    dx13=(double)x13/(double)y13;
  bool flat_top = y12==0;
  bool flat_bottom = y23==0;

  // For texture interpolation
  double u12 = tx2.x() - tx1.x();
  double u13 = tx3.x() - tx1.x();
  double v13 = tx3.y() - tx1.y();
  double v12 = tx2.y() - tx1.y();
  double w12 = tx2.z() - tx1.z();
  double w13 = tx3.z() - tx1.z();

  Vector3d pt12_3d = pt2_3d-pt1_3d;
  Vector3d pt13_3d = pt3_3d-pt1_3d;
  Vector3d pt23_3d = pt3_3d-pt2_3d;

  // _d as reminder that it's double
  double sx_d = p1x;
  double ex_d = p1x;
  // Multiple light sources are additive? The energy should be, but perceived
  // brightness may not be
  for (const auto &named_light : lights) {
    auto light = named_light.second;
    // Scan horizontal lines from top to bottom of triangle
    // This is for the top "half" of the triangle
    for (int dy = 0; p1y + dy <= p2y; ++dy) {
      // Flat-top triangle will be taken care of completely in bottom half
      // loop below.
      if (flat_top)break;
      double d_vertical12 = (double) dy / y12;
      double d_vertical13 = (double) dy / y13;

      // Select integer pixel indices using the double type values
      int sx = std::round(sx_d);
      int ex = std::round(ex_d);

      // Start and end of texel along this horizontal line
      double stx_u = tx1.x() + u12 * d_vertical12;
      double etx_u = tx1.x() + u13 * d_vertical13;
      double stx_v = tx1.y() + v12 * d_vertical12;
      double etx_v = tx1.y() + v13 * d_vertical13;
      double stx_w = tx1.z() + w12 * d_vertical12;
      double etx_w = tx1.z() + w13 * d_vertical13;

      // Start and end of normals
      Vector3d snorm = slerp(n1, n2, d_vertical12);
      Vector3d enorm = slerp(n1, n3, d_vertical13);

      // Start and end of 3d points in camera frame
      Vector3d s3d = pt1_3d + pt12_3d * d_vertical12;
      Vector3d e3d = pt1_3d + pt13_3d * d_vertical13;

      if (ex < sx) {
        std::swap(sx, ex);
        std::swap(stx_u, etx_u);
        std::swap(stx_v, etx_v);
        std::swap(stx_w, etx_w);
        std::swap(snorm, enorm);
      }
      for (int dx = 0; dx <= ex - sx; ++dx) {
        double d_horizontal = 0;
        // 3D point along the horizontal
        Vector3d pt3d = s3d + (e3d - s3d) * d_horizontal;
        // point normal along the horizontal
        Vector3d norm = slerp(snorm, enorm, d_horizontal);

        // direction from point to light source
        Vector3d dir_2_light = -light->getDirection(pt3d);
        // direction from point to camera
        Vector3d dir_2_cam = -pt3d; // point should already be in camera frame
        dir_2_cam.normalize();
        // Compute halfway vector between point to camera and point to light
        auto halfway = slerp(dir_2_light, dir_2_cam, 0.5);

        // Pick texel color
        if (sx != ex) d_horizontal = (double) dx / (double) (ex - sx);
        double interp_texel_w = stx_w + (etx_w - stx_w) * d_horizontal;

        Vector3d color(1, 1, 1);
        if (tri.material->texture != nullptr) {
          double interp_texel_u = stx_u + (etx_u - stx_u) * d_horizontal;
          double interp_texel_v = stx_v + (etx_v - stx_v) * d_horizontal;
          double real_u =
              std::min(1.0, std::max(0.0, interp_texel_u / interp_texel_w));
          double real_v =
              std::min(1.0, std::max(0.0, interp_texel_v / interp_texel_w));
          auto texture_color = tri.material->texture->Sample(real_u, real_v);
          double r = (double) texture_color.r / 255.0;
          double g = (double) texture_color.g / 255.0;
          double b = (double) texture_color.b / 255.0;
          color = Vector3d(r, g, b);
        }

        auto final_color = shade(tri.material, light, norm, dir_2_light,
                                 halfway, color);

        olc::Pixel px = olc::PixelF(final_color.x(),
                                    final_color.y(),
                                    final_color.z());

        int screen_x = sx + dx;
        int screen_y = p1y + dy;
        if (screen_x > pge->ScreenWidth() || screen_x < 0)continue;
        if (screen_y > pge->ScreenHeight() || screen_y < 0)continue;
        if (interp_texel_w > depth_buffer[screen_y][screen_x]) {
          pge->Draw(screen_x, screen_y, px);
          depth_buffer[screen_y][screen_x] = interp_texel_w;
        }
      }
      // Increment start and end x values
      sx_d += dx12;
      ex_d += dx13;
    }
    if (!flat_top) ex_d -= dx13;

    int x23 = p3x - p2x;
    double dx23 = 0;
    if (y23 != 0)
      dx23 = (double) x23 / (double) y23;

    double u23 = tx3.x() - tx2.x();
    double v23 = tx3.y() - tx2.y();
    double w23 = tx3.z() - tx2.z();

    sx_d = p2x;

    for (int dy = 0; dy <= p3y-p2y; ++dy) {
      // Bottom-flat triangle should have been completely drawn already by
      // the loop above.
      if (flat_bottom) break;
      double d_vertical23 = (double) dy / y23;
      double d_vertical13 = (double) (dy + y12) / y13;

      // Select actual pixel indices using the double type values
      int sx = std::round(sx_d);
      int ex = std::round(ex_d);

      // Start and end of texel along this horizontal line
      double stx_u = tx2.x() + u23 * d_vertical23;
      double etx_u = tx1.x() + u13 * d_vertical13;
      double stx_v = tx2.y() + v23 * d_vertical23;
      double etx_v = tx1.y() + v13 * d_vertical13;
      double stx_w = tx2.z() + w23 * d_vertical23;
      double etx_w = tx1.z() + w13 * d_vertical13;

      // Start and end of normals
      Vector3d snorm = slerp(n2, n3, d_vertical23);
      Vector3d enorm = slerp(n1, n3, d_vertical13);

      // Start and end of 3d points in camera frame
      Vector3d s3d = pt2_3d + pt23_3d * d_vertical23;
      Vector3d e3d = pt1_3d + pt13_3d * d_vertical13;

      if (ex < sx) {
        std::swap(sx, ex);
        std::swap(stx_u, etx_u);
        std::swap(stx_v, etx_v);
        std::swap(stx_w, etx_w);
        std::swap(snorm, enorm);
        std::swap(s3d, e3d);
      }

      for (int dx = 0; dx <= ex-sx; ++dx) {
        double d_horizontal = 0;
        if (sx != ex) d_horizontal = (double) dx / (double) (ex - sx);

        // 3D point along the horizontal
        Vector3d pt3d = s3d + (e3d - s3d) * d_horizontal;
        // point normal along the horizontal
        Vector3d norm = slerp(snorm, enorm, d_horizontal);

        // direction from point to light source
        Vector3d dir_2_light = -light->getDirection(pt3d);
        // direction from point to camera
        Vector3d dir_2_cam = -pt3d; // point should already be in camera frame
        dir_2_cam.normalize();
        // Compute halfway vector between point to camera and point to light
        auto halfway = slerp(dir_2_light, dir_2_cam, 0.5);

        // Pick texel color
        double interp_texel_w = stx_w + (etx_w - stx_w) * d_horizontal;

        Vector3d color(1, 1, 1);
        if (tri.material->texture != nullptr) {
          double interp_texel_u = stx_u + (etx_u - stx_u) * d_horizontal;
          double interp_texel_v = stx_v + (etx_v - stx_v) * d_horizontal;
          double real_u =
              std::min(1.0, std::max(0.0, interp_texel_u / interp_texel_w));
          double real_v =
              std::min(1.0, std::max(0.0, interp_texel_v / interp_texel_w));
          auto texture_color = tri.material->texture->Sample(real_u, real_v);
          double r = (double) texture_color.r / 255.0;
          double g = (double) texture_color.g / 255.0;
          double b = (double) texture_color.b / 255.0;
          color = Vector3d(r, g, b);
        }

        auto final_color = shade(tri.material, light, norm, dir_2_light,
                                 halfway, color);

        olc::Pixel px = olc::PixelF(final_color.x(),
                                    final_color.y(),
                                    final_color.z());

        int screen_x = sx + dx;
        int screen_y = p2y + dy;
        if (screen_x > pge->ScreenWidth() || screen_x < 0)continue;
        if (screen_y > pge->ScreenHeight() || screen_y < 0)continue;
        if (interp_texel_w > depth_buffer[screen_y][screen_x]) {
          pge->Draw(screen_x, screen_y, px);
          depth_buffer[screen_y][screen_x] = interp_texel_w;
        }
      }
      // Increment start and end x values
      sx_d += dx23;
      ex_d += dx13;
    }
  }
}

Vector3d Renderer::shade(
    const Vector3d &material_ambience, const Vector3d &material_diffuse,
    const Vector3d &material_specular, const Vector3d &surface_normal,
    const Vector3d &point_to_light, const Vector3d &cam_2_light_halfway,
    double glossiness_exponent, const Vector3d &material_emittance,
    const Vector3d &light_ambience, const Vector3d &light_diffuse,
    const Vector3d &light_specular, const Vector3d &color_from_texture){

  double n_dot_l = surface_normal.dot(point_to_light);

  Vector3d ambiance = material_ambience.cwiseProduct(light_ambience);
  Vector3d diffuse = material_diffuse.cwiseProduct(light_diffuse)
      * std::max(n_dot_l, 0.0);
  Vector3d amb_dif_emit = ambiance+diffuse+material_emittance;
  Vector3d with_texture = amb_dif_emit.cwiseProduct(color_from_texture);
  Vector3d I_rgb = with_texture;

  // Shouldn't have specular reflection if light is behind surface right??
  if(n_dot_l > 0){
    Vector3d spec = material_specular.cwiseProduct(light_specular)
    * pow(surface_normal.dot(cam_2_light_halfway),glossiness_exponent);
    I_rgb += spec;
  }

  // Clip rgb to max 1.0
  I_rgb.x() = std::min(I_rgb.x(),1.0);
  I_rgb.y() = std::min(I_rgb.y(),1.0);
  I_rgb.z() = std::min(I_rgb.z(),1.0);

  return I_rgb;
}

Vector3d Renderer::shade(const Material_ptr& material,
                         const Light_ptr& light_source,
                         Vector3d point_normal,
                         Vector3d point_to_light_vector,
                         Vector3d halfway_vec,
                         const Vector3d &color_from_texture){

  double point_norm = point_normal.norm();
  if(point_norm < cg::EPS && point_norm > -cg::EPS)
    throw std::runtime_error("Surface normal vector is length 0");
  if(point_norm<1-cg::EPS || point_norm > 1+cg::EPS)
    point_normal.normalize();

  double light_norm = point_to_light_vector.norm();
  if(light_norm < cg::EPS && light_norm > -cg::EPS)
    throw std::runtime_error("Point to light vector is length 0");
  if(light_norm<1-cg::EPS || light_norm> 1+cg::EPS)
    point_to_light_vector.normalize();

  double halfway_norm = halfway_vec.norm();
  if(halfway_norm < cg::EPS && halfway_norm > -cg::EPS)
    throw std::runtime_error("halfway vector is length 0");
  if(halfway_norm<1-cg::EPS || halfway_norm> 1+cg::EPS)
    halfway_vec.normalize();

  return shade(material->ka, material->kd, material->ks, point_normal,
               point_to_light_vector, halfway_vec, material->Ns, material->ke,
               light_source->La, light_source->Ld, light_source->Ls,
               color_from_texture);
}

} //namespace cg
