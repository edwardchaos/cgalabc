#include <cmath>
#include <memory>
#include <map>

#include <olcPixelGameEngine.h>

#include "Mesh.h"
#include "Utility.h"
#include "Camera.h"
#include "Renderer.h"

using Eigen::Vector3d;

class CameraApplication: public olc::PixelGameEngine{
 public:
  bool OnUserCreate() override{
    cams.emplace("ego", std::make_shared<cg::Camera>(M_PI/2.0,
                                                     0.01,
                                                     50,
                                                     ScreenWidth(),
                                                     ScreenHeight()));

    renderer = std::make_unique<cg::Renderer>(this);

//    meshes.emplace("santa", saint);
//    auto spyro= cg::loadOBJ("/home/shooshan/Pictures/spyro.obj", false);
//    meshes.emplace("spyro", spyro);
//    auto teddy= cg::loadOBJ("/home/shooshan/Pictures/teddy.obj", false);
//    meshes.emplace(teddy);
//    auto teapot = cg::loadOBJ("/home/shooshan/Pictures/teapot.obj", false);
//    meshes.emplace("teapot", teapot);
      meshes.emplace("cube", cg::cube());
//    cg::Triangle triangle{
//      Vector3d(0,-1,-10),Vector3d(-1,-1,-10),Vector3d(0,1,-10),
//      Vector2d(0,1),Vector2d(1,1),Vector2d(0,0)};
//    cg::Mesh_ptr tri_mesh;
//    tri_mesh->tris.push_back(triangle);
//    meshes.emplace("triangle" ,tri_mesh);

// 2 Thin triangles
//    cg::Triangle thin_bottom{
//      Vector3d(0,0,-10),
//      Vector3d(-10,0,-10),
//      Vector3d(-10,0.5,-10)};
//    cg::Triangle thin_top{
//      Vector3d(0,0,-10),
//      Vector3d(-10,0.5,-10),
//      Vector3d(0,0.5,-10)};
//
//    cg::Mesh_ptr thin_tri_mesh;
//    thin_tri_mesh->tris.push_back(thin_bottom);
//    thin_tri_mesh->tris.push_back(thin_top);
//    meshes.emplace("thin_triangles" ,thin_tri_mesh);

//    auto axis = cg::loadOBJ("/home/shooshan/Pictures/axis.obj", false);
//    meshes.emplace("axis",axis);

    return true;
  }

  bool OnUserUpdate(float fElapsedTime) override {
    // Modify mesh's world position
    double z_rot = 0.0001/fElapsedTime;
    double x_rot = 0.0001/fElapsedTime;
    double y_rot = 0.0001/fElapsedTime;

    auto rotx_mat = cg::rotateX(x_rot);
    auto roty_mat = cg::rotateY(y_rot);
    auto rotz_mat = cg::rotateZ(z_rot);

    renderer->clear();
    // Handle keyboard input
    handleCameraMotion(cams.at("ego"), fElapsedTime);

    for(auto &mesh : meshes) {
      mesh.second->pose.position = cg::translation(0, 0, -20).rightCols<1>();
      //mesh.pose.orientation = rotx_mat*roty_mat*rotz_mat*mesh.pose.orientation;
      //Eigen::Matrix4d tf = mesh.pose.matrix();
      Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();

      renderer->draw(cams.at("ego"), mesh.second);
    }

    return true;
  }

 private:
  std::map<std::string, cg::Camera_ptr> cams;
  std::map<std::string, cg::Mesh_ptr> meshes;
  std::unique_ptr<cg::Renderer> renderer;

  void handleCameraMotion(const cg::Camera_ptr &cam, double fElapsedTime){
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
};

int main(){
  CameraApplication app;
  if(!app.Construct(640, 480, 10, 10)) return 0;
  app.Start();
  return 0;
}