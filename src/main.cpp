#define OLC_PGE_APPLICATION

#include <vector>

#include "olcPixelGameEngine.h"
#include "Algo2d.h"

enum Dir{N,E,S,W};
typedef std::vector<int> vi;
typedef std::vector<bool> vb;

struct sCell{
  vi wall_id;
  vb wall;
  bool enabled;
  sCell(): enabled(false){
    wall_id.assign(4,0);
    wall.assign(4,false);
  }
};

class LineOfSight: public olc::PixelGameEngine{
 public:
  bool OnUserCreate() override{
    world_w_ = ScreenWidth() / cell_size_;
    world_h_ = ScreenHeight() / cell_size_;
    world_.assign(world_w_ * world_h_, sCell());

    // Create border around world
    for(int i = 0; i < world_w_; ++i){
      world_[i].enabled = true;
      world_[(world_h_-1)*world_w_+i].enabled = true;
    }
    for(int i = 1; i < world_h_-1; ++i){
      world_[i*world_w_].enabled = true;
      world_[i*world_w_ + world_w_-1].enabled = true;
    }

    return true;
  }

  bool OnUserUpdate(float fElapsedTime) override{

    // By default draw target is the window so this isn't necessary
    SetDrawTarget(nullptr);
    Clear(olc::DARK_GREY);

    handleMouseInput();
    drawWorld();

    return true;
  }

 private:
  int cell_size_ = 10;
  int world_w_, world_h_;
  std::vector<sCell> world_;

  void drawWorld() {
    for (int i = 0; i < world_w_; ++i) {
      for (int j = 0; j < world_h_; ++j) {
        int cell_idx = j * world_w_ + i;

        if (world_[cell_idx].enabled)
          FillRect(i * cell_size_, j * cell_size_, cell_size_, cell_size_,
                   olc::BLACK);
      }
    }
  }

  void handleMouseInput(){
    // Get mouse right click
    if(GetMouse(1).bHeld)
      DrawCircle(GetMousePos(), 10);

    // clicked
    if(GetMouse(0).bReleased){
      int mouse_x = GetMouseX();
      int mouse_y = GetMouseY();

      int cell_x = mouse_x / (int)cell_size_;
      int cell_y = mouse_y / (int)cell_size_;
      int cell_idx = cell_y * world_w_ + cell_x;

      // Don't allow user to toggle off world border
      if(cell_x==0 || cell_x==world_w_-1
      || cell_y==0 || cell_y==world_h_-1) return;

      world_[cell_idx].enabled = !world_[cell_idx].enabled;
    }
  }

};

int main(){
  LineOfSight engine;
  if(!engine.Construct(320, 240, 10,10)) return 0;
  engine.Start();
  return 0;
}
