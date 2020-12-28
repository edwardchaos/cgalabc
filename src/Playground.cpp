#define OLC_PGE_APPLICATION

#include <memory>

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
    mesh = *cg::cube();
//    cg::Triangle triangle{
//      Vector3d(0,-5,-10),Vector3d(-5,0,-20),
//      Vector3d(0,5,-10)};
//    mesh.tris.push_back(triangle);
//    auto axis = cg::loadOBJ("/home/shooshan/Pictures/axis.obj");
//    mesh = *axis;
    sprite.LoadFromFile("/home/shooshan/Pictures/free.png");

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
        pt0_tf.head<3>(), pt1_tf.head<3>(), pt2_tf.head<3>()};

      auto triangles_to_draw = cam->projectTriangleInWorld(tri_world);
      // Should return 2d triangles to draw along with 2d sprite coordinates.

      for(const auto& screen_tri : triangles_to_draw){
        //DrawTexturedTriangle(screen_tri, sprite);

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

//  void DrawTexturedTriangle(triangle, const olc::Sprite &sprite){
//    // Iterate from top to bottom of triangle
//
//    // Iterate across a horizontal line
//
//    // Get textile pixel value
//    auto color = sprite.Sample(0.5,0.5);
//
//    // Get screen x,y
//    int screen_x, screen_y;
//
//    Draw(screen_x, screen_y, color);
//  }
};

int main(){
  CameraApplication app;
  if(!app.Construct(640, 480, 10, 10)) return 0;
  app.Start();
  return 0;
}