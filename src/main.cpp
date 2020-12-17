#define OLC_PGE_APPLICATION

#include "olcPixelGameEngine.h"
#include "Algo2d.h"

class DisplayEngine : public olc::PixelGameEngine{
 public:
  bool OnUserCreate() override{
    return true;
  }
  bool OnUserUpdate(float fElapsedTime) override{
    // called once per frame
    for (int x = 0; x < ScreenWidth(); x++)
      for (int y = 0; y < ScreenHeight(); y++)
        Draw(x, y, olc::Pixel(rand() % 255, rand() % 255, rand()% 255));
    return true;
  }
};

int main(){
  DisplayEngine engine;
  if(!engine.Construct(640, 480, 1,1)) return 0;
  engine.Start();
  return 0;
}
