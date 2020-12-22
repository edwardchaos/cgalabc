#define OLC_PGE_APPLICATION

#include <memory>
#include <cassert>

#include <olcPixelGameEngine.h>

#include "include/Camera.h"
#include "include/Mesh.h"

using Eigen::Vector3d;

class CameraApplication: public olc::PixelGameEngine{
 public:
  bool OnUserCreate() override{
    cam = std::make_shared<cg::Camera>(
        (double)ScreenHeight()/(double)ScreenWidth(),
        M_PI/2.0,
        10,
        100);

    // create cube
    // South face
    cube.tris.emplace_back(Vector3d(0,0,0),Vector3d(1,1,0),Vector3d(0,1,0));
    cube.tris.emplace_back(Vector3d(0,0,0),Vector3d(1,0,0),Vector3d(1,1,0));

    // East face
    cube.tris.emplace_back(Vector3d(1,0,0),Vector3d(1,1,-1),Vector3d(1,1,0));
    cube.tris.emplace_back(Vector3d(1,0,0),Vector3d(1,0,-1),Vector3d(1,1,-1));

    // West face
    cube.tris.emplace_back(Vector3d(0,0,0),Vector3d(0,1,0),Vector3d(0,1,-1));
    cube.tris.emplace_back(Vector3d(0,0,0),Vector3d(0,1,-1),Vector3d(0,0,-1));

    // North face
    cube.tris.emplace_back(Vector3d(0,0,-1),Vector3d(0,1,-1),Vector3d(1,1,-1));
    cube.tris.emplace_back(Vector3d(0,0,-1),Vector3d(1,1,-1),Vector3d(1,0,-1));

    // Top face
    cube.tris.emplace_back(Vector3d(0,1,0),Vector3d(1,1,0),Vector3d(1,1,-1));
    cube.tris.emplace_back(Vector3d(0,1,0),Vector3d(1,1,-1),Vector3d(0,1,-1));

    // Bottom face
    cube.tris.emplace_back(Vector3d(0,0,0),Vector3d(0,0,-1),Vector3d(1,0,-1));
    cube.tris.emplace_back(Vector3d(0,0,0),Vector3d(1,0,-1),Vector3d(1,0,0));


    return true;
  }

  bool OnUserUpdate(float fElapsedTime) override {
    Clear(olc::DARK_GREY);

    static double z_rot = 0;
    static double x_rot = 0;
    static double y_rot = 0;

    Eigen::Matrix3d rotZ = Eigen::Matrix3d::Identity();
    rotZ(0,0) = cos(z_rot);
    rotZ(0,1) = -sin(z_rot);
    rotZ(1,0) = sin(z_rot);
    rotZ(1,1) = cos(z_rot);

    Eigen::Matrix3d rotX = Eigen::Matrix3d::Identity();
    rotX(1,1) = cos(x_rot);
    rotX(1,2) = -sin(x_rot);
    rotX(2,1) = sin(x_rot);
    rotX(2,2) = cos(x_rot);

    Eigen::Matrix3d rotY = Eigen::Matrix3d::Identity();
    rotY(0,0) = cos(x_rot);
    rotY(0,2) = sin(x_rot);
    rotY(2,0) = -sin(x_rot);
    rotY(2,2) = cos(x_rot);

    z_rot += 0.0001/fElapsedTime;
    x_rot += 0.0001/fElapsedTime;
    y_rot += 0.0001/fElapsedTime;

    cg::Mesh cube_copy = this->cube;
    // Rotate cube
    for(auto &tri : cube_copy.tris) for(auto &pt : tri.points){
      pt = (Eigen::RowVector3d(pt) * rotX * rotZ * rotY);
    }

    // Translate cube infront of camera
    for(auto &tri : cube_copy.tris) for(auto & pt : tri.points) pt(2) -= 3;

    // Project points of cube onto camera image
    for(const auto& tri : cube_copy.tris){
      // Only consider drawing triangle if it's facing the cam
      if(!cam->isFacing(tri)) continue;
      std::vector<cg::Point2d> tri_img_pts;

      for(const auto& pt : tri.points){
        // Extend point to homogenous coordinate
        Eigen::Vector4d homo_pt;
        homo_pt.head<3>() = pt;
        homo_pt(3) = 1;

        // project point
        auto normalized_img_pt = cam->projectPoint(homo_pt);

        // Point coordinate is in range [-1,1]. Expand it to the width and
        // height of the screen.
        double screen_x = (normalized_img_pt->x()+1)*ScreenWidth()/2.0;
        double screen_y = (normalized_img_pt->y()+1)*ScreenHeight()/2.0;
        tri_img_pts.emplace_back(screen_x, screen_y);
      }
      assert(tri_img_pts.size() == 3);

      // Draw lines of triangles
      for(int i = 0; i < tri_img_pts.size(); ++i){
        DrawLine(tri_img_pts[i].x(), tri_img_pts[i].y(),
                 tri_img_pts[(i+1)%3].x(), tri_img_pts[(i+1)%3].y());
      }
    }

    return true;
  }

 private:
  cg::Camera_ptr cam;
  cg::Mesh cube;
};

int main(){
  CameraApplication app;
  if(!app.Construct(640, 480, 10, 10)) return 0;
  app.Start();
  return 0;
}