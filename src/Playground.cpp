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
    Vector3d light_brightness(1.7,1.7,1.7);
    lights.at("dir_light")->La = light_brightness;
    lights.at("dir_light")->Ld = light_brightness;
    lights.at("dir_light")->Ls = light_brightness;

    renderer = std::make_unique<cg::Renderer>(this);

    //meshes.emplace("spyro", cg::spyro());
    //meshes.emplace("teddy", cg::teddy());
    //meshes.emplace("teapot", cg::teapot());
    //meshes.emplace("cube", cg::cube());
    //meshes.emplace("simple_tri", cg::simpleTriangle());
    //meshes.emplace("thin_triangles" ,cg::thinTriangles());
    //meshes.emplace("axis",cg::worldAxis());

//    auto res_path = cg::getResourcesPath();
//    auto axis_path = res_path + "sample_models/spyro.obj";
//    auto loaded_meshes = cg::tinyOBJLoad(axis_path,res_path+"sample_models/");
    std::string obj_path=
        "/home/shooshan/Documents/objs/spyro_artisans/artisans_hub.obj";
    std::string mtl_path=
        "/home/shooshan/Documents/objs/spyro_artisans/";
    auto loaded_meshes = cg::tinyOBJLoad(obj_path,mtl_path);
    for(const auto &loaded_mesh: loaded_meshes){
      meshes.emplace("test_load", loaded_mesh);
    }

    return true;
  }

  bool OnUserUpdate(float fElapsedTime) override {
//    double z_rot = 0.0001/fElapsedTime;
//    double x_rot = 0.0001/fElapsedTime;
//    double y_rot = 0.0001/fElapsedTime;
//
//    auto rotx_mat = cg::rotateX(x_rot);
//    auto roty_mat = cg::rotateY(y_rot);
//    auto rotz_mat = cg::rotateZ(z_rot);

    renderer->clear();
    // Handle keyboard input
    handleCameraMotion(cams.at("ego"), fElapsedTime);

    for(auto &mesh : meshes) {
      mesh.second->pose.position = cg::translation(0, -10, 50).rightCols<1>();
//      mesh.second->pose.orientation =
//          rotx_mat*roty_mat*rotz_mat*mesh.second->pose.orientation;

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
      cam->moveForward(0.1/fElapsedTime);
    }
    if(GetKey(olc::Key::S).bHeld){
      DrawString(50,50, "Strafe Left");
      cam->strafeRight(-0.1/fElapsedTime);
    }
    if(GetKey(olc::Key::D).bHeld){
      DrawString(50,50, "Back");
      cam->moveForward(-0.1/fElapsedTime);
    }
    if(GetKey(olc::Key::F).bHeld){
      DrawString(50,50, "Strafe Right");
      cam->strafeRight(0.1/fElapsedTime);
    }
    if(GetKey(olc::Key::H).bHeld){
      DrawString(50,50, "Yaw Left");
      cam->yawRight(-0.005/fElapsedTime);
    }
    if(GetKey(olc::Key::L).bHeld){
      DrawString(50,50, "Yaw Right");
      cam->yawRight(0.005/fElapsedTime);
    }
    if(GetKey(olc::Key::K).bHeld){
      DrawString(50,50, "Up");
      cam->moveUp(0.1/fElapsedTime);
    }
    if(GetKey(olc::Key::J).bHeld){
      DrawString(50,50, "Down");
      cam->moveUp(-0.1/fElapsedTime);
    }
  }
};

int main(){
  CameraApplication app;
  if(!app.Construct(640, 480, 10, 10)) return 0;
  app.Start();
  return 0;
}