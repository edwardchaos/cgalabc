#include <cmath>
#include <memory>
#include <unordered_map>

#include <olcPixelGameEngine.h>

#include "Mesh.h"
#include "Utility.h"
#include "Camera.h"
#include "Renderer.h"
#include "Light.h"

using Eigen::Vector3d;

class CameraApplication: public olc::PixelGameEngine{
 public:
  bool OnUserCreate() override{
    cams.emplace("ego", std::make_shared<cg::Camera>(M_PI/2.0,
                                                     0.01,
                                                     50,
                                                     ScreenWidth(),
                                                     ScreenHeight()));

    lights.emplace("dir_light",
                   std::make_shared<cg::DirectionLight>(Vector3d(1,-1,1)));

    renderer = std::make_unique<cg::Renderer>(this);

    //meshes.emplace("spyro", cg::spyro());
    //meshes.emplace("teddy", cg::teddy());
    meshes.emplace("teapot", cg::teapot());
    //meshes.emplace("cube", cg::cube());
    //meshes.emplace("simple_tri", cg::simpleTriangle());
    //meshes.emplace("thin_triangles" ,cg::thinTriangles());
    //meshes.emplace("axis",cg::worldAxis());

    return true;
  }

  bool OnUserUpdate(float fElapsedTime) override {
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
      mesh.second->pose.position = cg::translation(0, 0, 5).rightCols<1>();
      mesh.second->pose.orientation =
          rotx_mat*roty_mat*rotz_mat*mesh.second->pose.orientation;
      Eigen::Matrix4d tf = mesh.second->pose.matrix();
      //Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();

      // Render mesh in camera 'ego'
      renderer->draw(cams.at("ego"), mesh.second, lights);
    }

    return true;
  }

 private:
  std::unordered_map<std::string, cg::Camera_ptr> cams;
  std::unordered_map<std::string, cg::Mesh_ptr> meshes;
  std::unordered_map<std::string, cg::Light_ptr> lights;
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
  if(!app.Construct(1920, 1080, 1, 1)) return 0;
  app.Start();
  return 0;
}