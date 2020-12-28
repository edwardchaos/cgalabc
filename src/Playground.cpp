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
        0.01,
        50,
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
      Vector3d(0,-1,-10),Vector3d(-1,-1,-10),Vector3d(0,1,-10),
      Vector2d(0,1),Vector2d(1,1),Vector2d(0,0)};
  mesh.tris.push_back(triangle);
//    auto axis = cg::loadOBJ("/home/shooshan/Pictures/axis.obj");
//    mesh = *axis;

    sprite.LoadFromFile("/home/shooshan/Pictures/free.png");
    //sprite.LoadFromFile("/home/shooshan/Pictures/checkerboard.png");

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

    // Some variables used in math
    // Difference between screen vertices
    double y12 = pt2.y() - pt1.y();
    double x12 = pt2.x() - pt1.x();
    double y13 = pt3.y() - pt1.y();
    double x13 = pt3.x() - pt1.x();

    // Differences between texture vertices
    double u12 = tx2.x() - tx1.x();
    double v12 = tx2.y() - tx1.y();
    double u13 = tx3.x() - tx1.x();
    double v13 = tx3.y() - tx1.y();
    double w12 = tx2.z() - tx1.z();
    double w13 = tx3.z() - tx1.z();

    double dt12 = 0;
    double dt13 = 0;
    if(y12!=0) dt12=1.0/fabs(y12);
    if(y13!=0) dt13=1.0/fabs(y13);
    double t12 = 0;
    double t13 = 0;

    // Scan horizontal lines from top to bottom of triangle
    // This is for the top "half" of the triangle
    for(int dy=0; pt1.y()+dy<=pt2.y(); ++dy){
      if(y12 == 0)break; // Top edge of triangle is horizontal. nothing to draw.
      // Get start of horizontal line
      int sx = (int)(pt1.x() + x12*t12);

      // Get end of horizontal line
      int ex = (int)(pt1.x() + x13*t13);

      // Get start of line on texture
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
        double spr_x = s_tx + (e_tx-s_tx)*t_horizontal;
        double spr_y = s_ty + (e_ty-s_ty)*t_horizontal;
        double spr_w = s_tw + (e_tw-s_tw)*t_horizontal;
        auto px_color = spr.Sample(spr_x/spr_w,spr_y/spr_w);

        // Draw the pixel value from texture in the screen xy position
        double screen_x = sx + dx;
        double screen_y = pt1.y() + dy;
        Draw(screen_x, screen_y, px_color);
        t_horizontal += dt_horizontal;
      }
      t12 += dt12;
      t13 += dt13;
    }

    double y23 = pt3.y()-pt2.y();
    double x23 = pt3.x()-pt2.x();
    double u23 = tx3.x()-tx2.x();
    double v23 = tx3.y()-tx2.y();
    double w23 = tx3.z()-tx2.z();

    double dt23 = 0;
    if(y23!=0) dt23 = 1.0/fabs(y23);
    double t23 = 0;
    // Similar drawing as above, but for the bottom "Half" of the triangle now.
    for(int dy=0; pt2.y()+dy<=pt3.y(); ++dy){
      // Get start of horizontal line
      int sx = (int)(pt2.x() + x23*t23);

      // Get end of horizontal line
      int ex = (int)(pt1.x() + x13*t13);

      // Get start of line on texture
      double s_tx = tx2.x() + t23*u23;
      double s_ty = tx2.y() + t23*v23;
      double s_tw = tx2.z() + t23*w23;

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
        double spr_x = s_tx + (e_tx-s_tx)*t_horizontal;
        double spr_y = s_ty + (e_ty-s_ty)*t_horizontal;
        double spr_w = s_tw + (e_tw-s_tw)*t_horizontal;
        auto px_color = spr.Sample(spr_x/spr_w,spr_y/spr_w);

        // Draw the pixel value from texture in the screen xy position
        double screen_x = sx + dx;
        double screen_y = pt2.y() + dy;
        Draw(screen_x, screen_y, px_color);
        t_horizontal += dt_horizontal;
      }
      t23 += dt23;
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