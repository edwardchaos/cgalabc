#include "Renderer.h"

#include "Utility.h"

namespace cg{

Renderer::Renderer(olc::PixelGameEngine* p_pge){
  // A pointer to the main pge object so we can draw in this class
  pge = p_pge;

  depth_buffer.resize(pge->ScreenHeight()+1);
  for(auto &row:depth_buffer)
    row.assign(pge->ScreenWidth()+1, -1e9);

  //sprite.LoadFromFile("/home/shooshan/Pictures/free.png");
  //sprite.LoadFromFile("/home/shooshan/Pictures/checkerboard.png");
}

void Renderer::clear(){
  pge->Clear(olc::DARK_GREY);

  // reset depth buffer
  for(auto &row:depth_buffer)
    row.assign(pge->ScreenWidth()+1, -1e9);
}

void Renderer::draw(const Camera_ptr &cam, const Mesh_ptr &mesh){
  for(const auto& tri : mesh->tris){
    // Move original mesh to its world position
    auto tf = mesh->pose.matrix();
    cg::Triangle tri_world = cg::transformTriangle(tri, tf);

    auto triangles_to_draw = cam->projectTriangleInWorld(tri_world);

    for(const auto& screen_tri : triangles_to_draw){
      //cg::shadeAndDrawTriangle(this,screen_tri,lights);
      if(screen_tri.material && screen_tri.material->texture)
      drawTexturedTriangle(screen_tri, *(screen_tri.material->texture));
      else {
//        pge->DrawTriangle(screen_tri.points[0].x(),screen_tri.points[0].y(),
//                          screen_tri.points[1].x(),screen_tri.points[1].y(),
//                          screen_tri.points[2].x(),screen_tri.points[2].y());
      }
    }
  }
}

void Renderer::drawTexturedTriangle(
    const cg::Triangle2D &tri, const olc::Sprite &spr){
  // Sort triangle vertices from top to bottom
  auto pt1 = tri.points[0];
  auto pt2 = tri.points[1];
  auto pt3 = tri.points[2];

  int p1x = std::round(pt1.x());
  int p1y = std::round(pt1.y());
  int p2x = std::round(pt2.x());
  int p2y = std::round(pt2.y());
  int p3x = std::round(pt3.x());
  int p3y = std::round(pt3.y());

  auto tx1 = tri.t[0];
  auto tx2 = tri.t[1];
  auto tx3 = tri.t[2];

  if(p2y < p1y){
    std::swap(p2x,p1x);
    std::swap(p2y,p1y);
    std::swap(tx2,tx1);}
  if(p3y < p1y){
    std::swap(p3x,p1x);
    std::swap(p3y,p1y);
    std::swap(tx3,tx1);}
  if(p3y < p2y){
    std::swap(p3x,p2x);
    std::swap(p3y,p2y);
    std::swap(tx3,tx2);}

  int y12 = p2y-p1y; assert(y12>=0);
  int x12 = p2x-p1x;
  int y13 = p3y-p1y; assert(y13>=0);
  int x13 = p3x-p1x;
  int y23 = p3y-p2y; assert(y23 >= 0);
  double dx12 = 0;
  if(y12!=0)
    dx12=(double)x12/(double)y12;
  double dx13 = 0;
  if(y13!=0)
    dx13=(double)x13/(double)y13;
  bool flat_top = y12==0;
  bool flat_bottom = y23==0;

  double u12 = tx2.x() - tx1.x();
  double u13 = tx3.x() - tx1.x();
  double v13 = tx3.y() - tx1.y();
  double v12 = tx2.y() - tx1.y();
  double w12 = tx2.z() - tx1.z();
  double w13 = tx3.z() - tx1.z();

//    if(flat_top && flat_bottom){
//      // Triangle points are on a single line
//      int min_x = std::min(std::min(p1x,p2x),p3x);
//      int max_x = std::max(std::max(p1x,p2x),p3x);
//
//      for(int dx=0; min_x+dx<=max_x; ++dx){
//        // TODO: Consider texture
//        Draw(min_x+dx,p1y,olc::WHITE);
//      }
//    }

  // _d as reminder that it's double
  double sx_d = p1x;
  double ex_d = p1x;

  // Scan horizontal lines from top to bottom of triangle
  // This is for the top "half" of the triangle
  for(int dy=0; p1y+dy<=p2y; ++dy){
    // Flat-top triangle will be taken care of completely in bottom half
    // loop below.
    if(flat_top)break;

    // Select actual pixel indices using the double type values
    int sx = std::round(sx_d);
    int ex = std::round(ex_d);

    // Start and end of texel along this horizontal line
    double stx_u = tx1.x() + u12*((double)dy/y12);
    double etx_u = tx1.x() + u13*((double)dy/y13);
    double stx_v = tx1.y() + v12*((double)dy/y12);
    double etx_v = tx1.y() + v13*((double)dy/y13);
    double stx_w = tx1.z() + w12*((double)dy/y12);
    double etx_w = tx1.z() + w13*((double)dy/y13);

    if(ex < sx){
      std::swap(sx,ex);
      std::swap(stx_u,etx_u);
      std::swap(stx_v,etx_v);
      std::swap(stx_w,etx_w);
    }

    for(int dx=0; sx+dx <= ex; ++dx){
      // Pick texel color
      double d_horizontal=0;
      if(sx!=ex) d_horizontal = (double)dx/(double)(ex-sx);
      double interp_texel_u = stx_u + (etx_u-stx_u)*d_horizontal;
      double interp_texel_v = stx_v + (etx_v-stx_v)*d_horizontal;
      double interp_texel_w = stx_w + (etx_w-stx_w)*d_horizontal;
      double real_u = std::min(1.0,std::max(0.0,interp_texel_u/interp_texel_w));
      double real_v = std::min(1.0,std::max(0.0,interp_texel_v/interp_texel_w));
      auto px_color = spr.Sample(real_u,real_v);

      int screen_x = sx + dx;
      int screen_y = p1y + dy;
      if(screen_x>pge->ScreenWidth()||screen_x<0)continue;
      if(screen_y>pge->ScreenHeight()||screen_y<0)continue;
      if(interp_texel_w > depth_buffer[screen_y][screen_x]){
        pge->Draw(screen_x, screen_y, px_color);
        depth_buffer[screen_y][screen_x] = interp_texel_w;
      }
    }
    // Increment start and end x values
    sx_d += dx12;
    ex_d += dx13;
  }
  if(!flat_top) ex_d -= dx13;

  int x23 = p3x-p2x;
  double dx23 = 0;
  if(y23!=0)
    dx23=(double)x23/(double)y23;

  double u23 = tx3.x() - tx2.x();
  double v23 = tx3.y() - tx2.y();
  double w23 = tx3.z() - tx2.z();

  sx_d = p2x;

  for(int dy=0; p2y+dy<=p3y; ++dy){
    // Bottom-flat triangle should have been completely drawn already by
    // the loop above.
    if(flat_bottom) break;

    // Select actual pixel indices using the double type values
    int sx = std::round(sx_d);
    int ex = std::round(ex_d);

    // Start and end of texel along this horizontal line
    double stx_u = tx2.x() + u23*((double)dy/y23);
    double etx_u = tx1.x() + u13*((double)(dy+y12)/y13);
    double stx_v = tx2.y() + v23*((double)dy/y23);
    double etx_v = tx1.y() + v13*((double)(dy+y12)/y13);
    double stx_w = tx2.z() + w23*((double)dy/y23);
    double etx_w = tx1.z() + w13*((double)(dy+y12)/y13);

    if(ex < sx){
      std::swap(sx,ex);
      std::swap(stx_u,etx_u);
      std::swap(stx_v,etx_v);
      std::swap(stx_w,etx_w);
    }

    for(int dx=0; sx+dx <= ex; ++dx){
      // Pick texel color
      double d_horizontal=0;
      if(sx!=ex) d_horizontal = (double)dx/(double)(ex-sx);
      double interp_texel_u = stx_u + (etx_u-stx_u)*d_horizontal;
      double interp_texel_v = stx_v + (etx_v-stx_v)*d_horizontal;
      double interp_texel_w = stx_w + (etx_w-stx_w)*d_horizontal;
      double real_u = std::min(1.0,std::max(0.0,interp_texel_u/interp_texel_w));
      double real_v = std::min(1.0,std::max(0.0,interp_texel_v/interp_texel_w));
      auto px_color = spr.Sample(real_u,real_v);

      int screen_x = sx + dx;
      int screen_y = p2y + dy;
      if(screen_x>pge->ScreenWidth()||screen_x<0)continue;
      if(screen_y>pge->ScreenHeight()||screen_y<0)continue;
      if(interp_texel_w > depth_buffer[screen_y][screen_x]){
        pge->Draw(screen_x, screen_y, px_color);
        depth_buffer[screen_y][screen_x] = interp_texel_w;
      }
    }
    // Increment start and end x values
    sx_d += dx23;
    ex_d += dx13;
  }
}
} //namespace cg
