#define OLC_PGE_APPLICATION

#include <memory>
#include <algorithm>

#include <olcPixelGameEngine.h>

#include "Mesh.h"
#include "Utility.h"
#include "Camera.h"

using Eigen::Vector3d;

class CameraApplication: public olc::PixelGameEngine{
 public:
  bool OnUserCreate() override{
    cam = std::make_shared<cg::Camera>(
        M_PI/2.0,
        0.1,
        100,
        ScreenWidth(),
        ScreenHeight());

//    auto spyro= cg::loadOBJ("/home/shooshan/Pictures/spyro.obj", true);
//    mesh = *spyro;
//    auto teddy= cg::loadOBJ("/home/shooshan/Pictures/teddy.obj", false);
//    mesh = *teddy;
//    auto teapot = cg::loadOBJ("/home/shooshan/Pictures/teapot.obj", false);
//    mesh = *teapot;
    //mesh = *cg::cube();
    cg::Triangle triangle{
      Vector3d(0,0,-10),Vector3d(-1,0,-10),
      Vector3d(0,1,-10),
      Vector2d(0,1),Vector2d(1,1),Vector2d(0,0)};
    mesh.tris.push_back(triangle);
//    auto axis = cg::loadOBJ("/home/shooshan/Pictures/axis.obj");
//    mesh = *axis;

    //sprite.LoadFromFile("/home/shooshan/Pictures/free.png");
    sprite.LoadFromFile("/home/shooshan/Pictures/checkerboard.png");

    return true;
  }

  bool OnUserUpdate(float fElapsedTime) override {
    Clear(olc::DARK_GREY);

    // Modify mesh's world position
    double z_rot = 0.0001/fElapsedTime;
    double x_rot = 0.0001/fElapsedTime;
    double y_rot = 0.0001/fElapsedTime;

    auto rotx_mat = cg::rotateX(x_rot);
    auto roty_mat = cg::rotateY(y_rot);
    auto rotz_mat = cg::rotateZ(z_rot);
    mesh.pose.position = cg::translation(0,0,-10).rightCols<1>();
    mesh.pose.orientation = rotx_mat*roty_mat*rotz_mat*mesh.pose.orientation;
    //Eigen::Matrix4d tf = mesh.pose.matrix();
    Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();

    // Handle keyboard input
    handleCameraMotion(fElapsedTime);

    for(const auto& tri : mesh.tris){
      // Move original mesh to its world position
      Eigen::Vector4d pt0_tf = cg::transformPoint(tri.points[0], tf);
      Eigen::Vector4d pt1_tf = cg::transformPoint(tri.points[1], tf);
      Eigen::Vector4d pt2_tf = cg::transformPoint(tri.points[2], tf);
      cg::Triangle tri_world{
        pt0_tf.head<3>(), pt1_tf.head<3>(), pt2_tf.head<3>(),
            tri.t[0].head<2>(), tri.t[1].head<2>(), tri.t[2].head<2>()};

      auto triangles_to_draw = cam->projectTriangleInWorld(tri_world);
      // Should return 2d triangles to draw along with 2d sprite coordinates.

      for(const auto& screen_tri : triangles_to_draw){
        DrawTexturedTriangle(screen_tri, sprite);

        DrawTriangle(screen_tri.points[0].x(), screen_tri.points[0].y(),
                     screen_tri.points[1].x(), screen_tri.points[1].y(),
                     screen_tri.points[2].x(), screen_tri.points[2].y());
      }
    }

    return true;
  }

 private:
  cg::Camera_ptr cam;
  cg::Mesh mesh;
  olc::Sprite sprite;

  void handleCameraMotion(double fElapsedTime){
    if(GetKey(olc::Key::E).bHeld){
      DrawString(50,50, "Forward");
      cam->moveForward(0.001/fElapsedTime);
    }
    if(GetKey(olc::Key::S).bHeld){
      DrawString(50,50, "Strafe Left");
      cam->strafeRight(-0.001/fElapsedTime);
    }
    if(GetKey(olc::Key::D).bHeld){
      DrawString(50,50, "Back");
      cam->moveForward(-0.001/fElapsedTime);
    }
    if(GetKey(olc::Key::F).bHeld){
      DrawString(50,50, "Strafe Right");
      cam->strafeRight(0.001/fElapsedTime);
    }
    if(GetKey(olc::Key::J).bHeld){
      DrawString(50,50, "Yaw Left");
      cam->yawRight(-0.0005/fElapsedTime);
    }
    if(GetKey(olc::Key::K).bHeld){
      DrawString(50,50, "Yaw Right");
      cam->yawRight(0.0005/fElapsedTime);
    }
  }

  void DrawTexturedTriangle(const cg::Triangle2D &tri, const olc::Sprite &spr){
    // Sort triangle vertices from top to bottom
    auto pt1 = tri.points[0];
    auto pt2 = tri.points[1];
    auto pt3 = tri.points[2];

    auto tx1 = tri.t[0];
    auto tx2 = tri.t[1];
    auto tx3 = tri.t[2];

    if(pt2.y() < pt1.y()){
      std::swap(pt2,pt1);
      std::swap(tx2,tx1);
    }
    if(pt3.y() < pt1.y()){
      std::swap(pt3,pt1);
      std::swap(tx3,tx1);
    }
    if(pt3.y() < pt2.y()){
      std::swap(pt3,pt2);
      std::swap(tx3,tx2);
    }

    // Scan horizontal lines from top to bottom of triangle

    // Acronyms short for variables in my math equations

    // The absolute differences between points
    double y12 = pt2.y() - pt1.y();
    double x12 = pt2.x() - pt1.x();
    double y13 = pt3.y() - pt1.y();
    double x13 = pt3.x() - pt1.x();
    double y12_abs = fabs(y12);
    double y13_abs = fabs(y13);

    // Texture coordinates
    double u12 = tx2.x() - tx1.x();
    double v12 = tx2.y() - tx1.y();
    double u13 = tx3.x() - tx1.x();
    double v13 = tx3.y() - tx1.y();
    double w12 = tx2.z() - tx1.z(); // Vector2d z() is index [2] (x,y,w)
    double w13 = tx3.z() - tx1.z();

    // The incremental deltas for scanning through triangle area
    double dx12 = 0;
    if(y12 != 0) dx12 = x12/y12;
    double dy12 = 1; //1 pixel down at a time.
    double dx13 = 0;
    if(y13 != 0) dx13 = x13/y13;
    double dy13 = 1; //1 pixel down at a time.

    double du12 = 0;
    if(y12!=0) du12 = u12/y12_abs;
    double dv12 = 0;
    if(y12!=0) dv12 = v12/y12_abs;
    double dw12 = 0;
    if(y12!=0) dw12 = w12/y12_abs;
    double du13 = 0;
    if(y13!=0) du13 = u13/y13_abs;
    double dv13 = 0;
    if(y13!=0) dv13 = v13/y13_abs;
    double dw13 = 0;
    if(y13!=0) dw13 = w13/y13_abs;

    double dt12 = 0;
    double dt13 = 0;
    if(y12!=0) dt12=1.0/y12_abs;
    if(y13!=0) dt13=1.0/y13_abs;
    double t12 = 0;
    double t13 = 0;

    for(int dy=0; pt1.y()+dy<=pt2.y(); ++dy){
      if(y12 == 0)break; // Top edge of triangle is horizontal. nothing to draw.
      // Get start of line
      int sx = (int)(pt1.x() + x12*t12);
      int sy = (int)(pt1.y()+dy);

      // Get end of line
      int ex = (int)(pt1.x() + x13*t13);
      int ey = sy;

      // Get start line on texture
      double s_tx = tx1.x() + t12*u12;
      double s_ty = tx1.y() + t12*v12;
      double s_tw = tx1.z() + t12*w12;

      // Get end of line on texture
      double e_tx = tx1.x() + t13*u13;
      double e_ty = tx1.y() + t13*v13;
      double e_tw = tx1.z() + t13*w13;

      // Drawing from left to right, swap start and end if they're reversed.
      if(ex <= sx){
        std::swap(sx,ex);
        // sy and ey are the same since it's a horizontal line

        // Also swap texture start/end
        std::swap(s_tx,e_tx);
        std::swap(s_ty, e_ty);
        std::swap(s_tw, e_tw);
      }

      // Draw along the horizontal line from start to end
      double t_horizontal = 0;
      double dt_horizontal = 0;
      if(ex-sx != 0) dt_horizontal = 1.0/abs(ex-sx);

      for(int dx=0; sx+dx <= ex; ++dx){
        // Get texture pixel value
        double screen_x = sx + dx;
        double screen_y = pt1.y() + dy;
        double spr_x = s_tx + (e_tx-s_tx)*t_horizontal;
        double spr_y = s_ty + (e_ty-s_ty)*t_horizontal;
        double spr_w = s_tw + (e_tw-s_tw)*t_horizontal;
        //auto px_color = spr.Sample(spr_x/spr_w,spr_y/spr_w);
        auto px_color = spr.Sample(spr_x,spr_y);

        Draw(screen_x, screen_y, px_color);
        t_horizontal += dt_horizontal;
      }
      t12 += dt12;
      t13 += dt13;
    }
  }
};

int main(){
  CameraApplication app;
  if(!app.Construct(640, 480, 10, 10)) return 0;
  app.Start();
  return 0;
}