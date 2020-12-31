#define OLC_PGE_APPLICATION

#include <cmath>
#include <memory>
#include <iostream>

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

    depth_buffer.resize(ScreenHeight()+1);
    for(auto &row:depth_buffer)
      row.assign(ScreenWidth()+1, -1e9);

//    auto saint= cg::loadOBJ(
//        "/home/shooshan/Documents/objs/Santa_Claus_v1_L2"
//        ".123cb4fe07b2-55dd-461b-9f27-42ccf1b3f3f5/santaclaus.obj", false);
//    mesh = *saint;
//    auto spyro= cg::loadOBJ("/home/shooshan/Pictures/spyro.obj", false);
//    mesh = *spyro;
//    auto teddy= cg::loadOBJ("/home/shooshan/Pictures/teddy.obj", false);
//    mesh = *teddy;
    auto teapot = cg::loadOBJ("/home/shooshan/Pictures/teapot.obj", false);
    mesh = *teapot;
      //mesh = *cg::cube();
//    cg::Triangle triangle{
//      Vector3d(0,-1,-10),Vector3d(-1,-1,-10),Vector3d(0,1,-10),
//      Vector2d(0,1),Vector2d(1,1),Vector2d(0,0)};
//    mesh.tris.push_back(triangle);

// 2 Thin triangles
//cg::Triangle thin_bottom{
//  Vector3d(0,0,-10),Vector3d(-10,0,-10),Vector3d(-10,0.5,-10)
//};
//mesh.tris.push_back(thin_bottom);
//    cg::Triangle thin_top{
//        Vector3d(0,0,-10),Vector3d(-10,0.5,-10),Vector3d(0,0.5,-10)
//    };
//    mesh.tris.push_back(thin_top);
//    auto axis = cg::loadOBJ("/home/shooshan/Pictures/axis.obj", false);
//    mesh = *axis;

    sprite.LoadFromFile("/home/shooshan/Pictures/rainbow.png");
    //sprite.LoadFromFile("/home/shooshan/Pictures/free.png");
    //sprite.LoadFromFile("/home/shooshan/Pictures/checkerboard.png");

    return true;
  }

  bool OnUserUpdate(float fElapsedTime) override {
    Clear(olc::DARK_GREY);
    // reset depth buffer
    for(auto &row:depth_buffer)
      row.assign(ScreenWidth(), -1e9);

    // Modify mesh's world position
    double z_rot = 0.0001/fElapsedTime;
    double x_rot = 0.0001/fElapsedTime;
    double y_rot = 0.0001/fElapsedTime;

    auto rotx_mat = cg::rotateX(x_rot);
    auto roty_mat = cg::rotateY(y_rot);
    auto rotz_mat = cg::rotateZ(z_rot);
    mesh.pose.position = cg::translation(0,0,-20).rightCols<1>();
    //mesh.pose.orientation = rotx_mat*roty_mat*rotz_mat*mesh.pose.orientation;
    Eigen::Matrix4d tf = mesh.pose.matrix();
    //Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();

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

      for(const auto& screen_tri : triangles_to_draw){
        DrawTexturedTriangle(screen_tri, sprite);

//        FillTriangle(screen_tri.points[0].x(), screen_tri.points[0].y(),
//                     screen_tri.points[1].x(), screen_tri.points[1].y(),
//                     screen_tri.points[2].x(), screen_tri.points[2].y());
      }
    }

    return true;
  }

 private:
  cg::Camera_ptr cam;
  cg::Mesh mesh;
  olc::Sprite sprite;
  std::vector<std::vector<double>> depth_buffer;

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

//    DrawCircle(p1x,p1y,2,olc::DARK_BLUE);
//    DrawCircle(p2x,p2y,2,olc::DARK_GREEN);
//    DrawCircle(p3x,p3y,2,olc::DARK_RED);

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

    if(flat_top && flat_bottom){
      // Triangle points are on a single line
      int min_x = std::min(std::min(p1x,p2x),p3x);
      int max_x = std::max(std::max(p1x,p2x),p3x);

      for(int dx=0; min_x+dx<=max_x; ++dx){
        Draw(min_x+dx,p1y,olc::WHITE);
      }
    }

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
      if(ex < sx){std::swap(sx,ex);}

      for(int dx=0; sx+dx <= ex; ++dx){
        int screen_x = sx + dx;
        int screen_y = p1y + dy;
        Draw(screen_x, screen_y, olc::WHITE);
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

    sx_d = p2x;

    for(int dy=0; p2y+dy<=p3y; ++dy){
      // Bottom-flat triangle should have been completely drawn already by
      // the loop above.
      if(flat_bottom) break;

      // Select actual pixel indices using the double type values
      int sx = std::round(sx_d);
      int ex = std::round(ex_d);
      if(ex < sx){std::swap(sx,ex);}

      for(int dx=0; sx+dx <= ex; ++dx){
        int screen_x = sx + dx;
        int screen_y = p2y + dy;
        Draw(screen_x, screen_y, olc::WHITE);
      }
      // Increment start and end x values
      sx_d += dx23;
      ex_d += dx13;
    }
  }
};

int main(){
  CameraApplication app;
  if(!app.Construct(640, 480, 10, 10)) return 0;
  app.Start();
  return 0;
}