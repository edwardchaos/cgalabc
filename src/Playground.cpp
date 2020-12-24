#define OLC_PGE_APPLICATION

#include <memory>
#include <cassert>

#include <olcPixelGameEngine.h>

#include "Mesh.h"
#include "Utility.h"
#include "Camera.h"

using Eigen::Vector3d;

class CameraApplication: public olc::PixelGameEngine{
 public:
  bool OnUserCreate() override{
    cam = std::make_shared<cg::Camera>(
        (double)ScreenHeight()/(double)ScreenWidth(),
        M_PI/2.0,
        5,
        100);

//    auto teapot = cg::loadOBJ("/home/shooshan/Pictures/teapot.obj");
//    mesh = *teapot;
//    mesh = *cg::cube();
    cg::Triangle triangle{
      Vector3d(0,-5,-10),Vector3d(-5,0,-20),
      Vector3d(0,5,-10)};
    mesh.tris.push_back(triangle);
//    auto axis = cg::loadOBJ("/home/shooshan/Pictures/axis.obj");
//    mesh = *axis;

    return true;
  }

  bool OnUserUpdate(float fElapsedTime) override {
    Clear(olc::DARK_GREY);

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

    //Handle move camera input
    handleCameraMotion(fElapsedTime);

    for(const auto& tri : mesh.tris){
      // Transform the triangle
      Eigen::Vector3d pt0_tf = cg::transformPoint(tri.points[0], tf);
      Eigen::Vector3d pt1_tf = cg::transformPoint(tri.points[1], tf);
      Eigen::Vector3d pt2_tf = cg::transformPoint(tri.points[2], tf);
      cg::Triangle tri_world{pt0_tf, pt1_tf, pt2_tf};

      // Only consider drawing triangle if it's facing the cam
      //if(!cam->isFacing(tri_world)) continue;

      // Transform triangle to cam space
      auto tri_cam = cam->tfTriangleWorldToCam(tri_world);

      // Clip triangle in cam coordinate frame by near plane
      auto near_clipped_tris_cam = cam->clipNear(tri_cam);

      for(auto &tri_cam : near_clipped_tris_cam){
        std::vector<Eigen::Vector2d> tri_img_pts;
        tri_img_pts.reserve(3);
        for(auto pt_cam : tri_cam.points){
          // Project triangle in camera frame onto camera's image plane.
          auto pt_cube = cam->projectPointInCameraFrame(pt_cam);

          // Scale triangles to screen size
          double screen_x = (pt_cube.x()+1)*ScreenWidth()/2.0;
          double screen_y = (-pt_cube.y()+1)*ScreenHeight()/2.0;
          tri_img_pts.emplace_back(screen_x,screen_y);
        }
        DrawTriangle(tri_img_pts[0].x(), tri_img_pts[0].y(),
                     tri_img_pts[1].x(), tri_img_pts[1].y(),
                     tri_img_pts[2].x(), tri_img_pts[2].y());
      }
    }

    return true;
  }

 private:
  cg::Camera_ptr cam;
  cg::Mesh mesh;

  void handleCameraMotion(double fElapsedTime){
    if(GetKey(olc::Key::E).bHeld){
      DrawString(50,50, "Forward");
      cam->moveForward(0.001/fElapsedTime);
    }
    if(GetKey(olc::Key::S).bHeld){
      DrawString(50,50, "Strafe Left");
      cam->strafeRight(-0.005/fElapsedTime);
    }
    if(GetKey(olc::Key::D).bHeld){
      DrawString(50,50, "Back");
      cam->moveForward(-0.001/fElapsedTime);
    }
    if(GetKey(olc::Key::F).bHeld){
      DrawString(50,50, "Strafe Right");
      cam->strafeRight(0.005/fElapsedTime);
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
};

int main(){
  CameraApplication app;
  if(!app.Construct(640, 480, 10, 10)) return 0;
  app.Start();
  return 0;
}