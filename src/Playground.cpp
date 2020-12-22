#define OLC_PGE_APPLICATION
#include <iostream>

#include <Eigen/Dense>
#include <olcPixelGameEngine.h>

#include "type.h"
#include "Camera.h"

class CameraApplication: public olc::PixelGameEngine{
 public:
  bool OnUserCreate() override{
  }

  bool OnUserUpdate(float fElapsedTime) override {
  }

 private:

};

int main(){
  CameraApplication app;
  if(!app.Construct(640, 480, 10, 10)) return 0;
  app.Start();
  return 0;
}