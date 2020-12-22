#define OLC_PGE_APPLICATION

#include <memory>

#include <olcPixelGameEngine.h>

#include "Camera.h"

class CameraApplication: public olc::PixelGameEngine{
 public:
  bool OnUserCreate() override{
    cam = std::make_shared<cg::Camera>(
        (double)ScreenHeight()/(double)ScreenWidth(),
        M_PI/2.0,
        10,
        1000);

    return true;
  }

  bool OnUserUpdate(float fElapsedTime) override {

    return true;
  }

 private:
  cg::Camera_ptr cam;

};

int main(){
  CameraApplication app;
  if(!app.Construct(640, 480, 10, 10)) return 0;
  app.Start();
  return 0;
}