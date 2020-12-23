#define OLC_PGE_APPLICATION

#include <memory>
#include <cassert>

#include <olcPixelGameEngine.h>

#include "Camera.h"
#include "Mesh.h"
#include "Utility.h"

using Eigen::Vector3d;

class CameraApplication: public olc::PixelGameEngine{
 public:
  bool OnUserCreate() override{
    cam = std::make_shared<cg::Camera>(
        (double)ScreenHeight()/(double)ScreenWidth(),
        M_PI/2.0,
        10,
        100);

    auto teapot = cg::loadOBJ("/home/shooshan/Pictures/teapot.obj");
    mesh = *teapot;
    //mesh = *cg::cube();

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
    Eigen::Matrix4d tf = mesh.pose.matrix();

    for(const auto& tri : mesh.tris){
      // Transform the triangle
      Eigen::Vector3d pt0_tf = cg::transformPoint(tri.points[0], tf);
      Eigen::Vector3d pt1_tf = cg::transformPoint(tri.points[1], tf);
      Eigen::Vector3d pt2_tf = cg::transformPoint(tri.points[2], tf);
      cg::Triangle tri_tf{pt0_tf, pt1_tf, pt2_tf};

      // Only consider drawing triangle if it's facing the cam
      if(!cam->isFacing(tri_tf)) continue;
      std::vector<cg::Point2d> tri_img_pts;
      tri_img_pts.reserve(3);

      for(const auto& pt : tri_tf.points){
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

      DrawTriangle(tri_img_pts[0].x(), tri_img_pts[0].y(),
                   tri_img_pts[1].x(), tri_img_pts[1].y(),
                   tri_img_pts[2].x(), tri_img_pts[2].y());
    }

    return true;
  }

 private:
  cg::Camera_ptr cam;
  cg::Mesh mesh;
};

int main(){
  CameraApplication app;
  if(!app.Construct(640, 480, 10, 10)) return 0;
  app.Start();
  return 0;
}