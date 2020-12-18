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
    convertWorldCellsToEdges();
    drawWorldEdges();

    return true;
  }

 private:
  int cell_size_ = 10;
  int world_w_, world_h_;
  std::vector<sCell> world_;
  std::vector<cg::Line2d> edges_;

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

  void drawWorldEdges(){
    for(const auto &e : edges_){
      DrawLine(e.pt1().x(),e.pt1().y(),e.pt2().x(),e.pt2().y());
      DrawCircle(e.pt1().x(),e.pt1().y(), 2, olc::BLUE);
      DrawCircle(e.pt2().x(),e.pt2().y(), 2, olc::BLUE);
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

  void convertWorldCellsToEdges(){
    edges_.clear();
    for(int j = 0; j < world_h_; ++j){
      for(int i = 0; i < world_w_; ++i){
        // reset all wall flags for this cell
        world_[cellIdx(i,j)].wall[N] = false;
        world_[cellIdx(i,j)].wall[S] = false;
        world_[cellIdx(i,j)].wall[E] = false;
        world_[cellIdx(i,j)].wall[W] = false;

        if(!world_[cellIdx(i,j)].enabled) continue;

        // Northern edge
        if(j-1>=0 && world_[cellIdx(i,j-1)].enabled){
          // Northern cell is occupied. No need for a northern edge.
        }else{
          // Try to extend the northern edge of the west neighbour if there
          // is one.
          if(i-1>=0
          && world_[cellIdx(i-1,j)].enabled
          && world_[cellIdx(i-1,j)].wall[N]){
            int edges_idx = world_[cellIdx(i-1,j)].wall_id[N];
            auto endpoint = edges_[edges_idx].pt2();

            // Extend the wall to the right by 1 cell
            edges_[edges_idx] = cg::Line2d(
                edges_[edges_idx].pt1(), // Original edge start point
                cg::Point2d(endpoint.x()+cell_size_, endpoint.y()));

            world_[cellIdx(i,j)].wall[N] = true;
            world_[cellIdx(i,j)].wall_id[N] = edges_idx;
          }else{
            // No occupied western neighbour cell. Start a new edge
            cg::Point2d startpoint(i*cell_size_,j*cell_size_);
            cg::Point2d endpoint(i*cell_size_+cell_size_, j*cell_size_);
            edges_.emplace_back(startpoint, endpoint);

            // Link the edge with the cell
            world_[cellIdx(i,j)].wall[N] = true;
            world_[cellIdx(i,j)].wall_id[N] = edges_.size()-1;
          }
        }

        // Western edge
        if(i-1>=0 && world_[cellIdx(i-1,j)].enabled){
          // Western cell is occupied. No need for a western edge
        }else{
          // Try to extend the western edge of the northern neighbour if there
          // is one.
          if(j-1>=0
          && world_[cellIdx(i,j-1)].enabled
          && world_[cellIdx(i,j-1)].wall[W]){
            int edges_idx = world_[cellIdx(i,j-1)].wall_id[W];
            auto endpoint = edges_[edges_idx].pt2();

            // Extend the endpoint down by 1 cell
            edges_[edges_idx] = cg::Line2d(
                edges_[edges_idx].pt1(), // Original edge start point
                cg::Point2d(endpoint.x(), endpoint.y()+cell_size_));

            world_[cellIdx(i,j)].wall[W] = true;
            world_[cellIdx(i,j)].wall_id[W] = edges_idx;
          }else{
            // No occupied northern neighbour cell. Start a new edge
            cg::Point2d startpoint(i*cell_size_,j*cell_size_);
            cg::Point2d endpoint(i*cell_size_, j*cell_size_+cell_size_);
            cg::Line2d new_edge(startpoint, endpoint);
            edges_.push_back(new_edge);

            // Link the edge with the cell
            world_[cellIdx(i,j)].wall[W] = true;
            world_[cellIdx(i,j)].wall_id[W] = edges_.size()-1;
          }
        }

        // Southern edge
        if(j+1<world_h_ && world_[cellIdx(i,j+1)].enabled){
          // Southern cell is occupied. No need for a southern edge
        }else{
          // Try to extend the southern edge of the western neighbour if there
          // is one.
          if(i-1>=0
          && world_[cellIdx(i-1,j)].enabled
          && world_[cellIdx(i-1,j)].wall[S]){
            int edges_idx = world_[cellIdx(i-1,j)].wall_id[S];
            auto endpoint = edges_[edges_idx].pt2();

            // Extend the endpoint east by 1 cell
            edges_[edges_idx] = cg::Line2d(
                edges_[edges_idx].pt1(), // Original edge start point
                cg::Point2d(endpoint.x()+cell_size_, endpoint.y()));

            world_[cellIdx(i,j)].wall[S] = true;
            world_[cellIdx(i,j)].wall_id[S] = edges_idx;
          }else{
            // No occupied western neighbour cell. Start a new edge
            cg::Point2d startpoint(i*cell_size_,j*cell_size_+cell_size_);
            cg::Point2d endpoint(i*cell_size_+cell_size_,
                                 j*cell_size_+cell_size_);
            cg::Line2d new_edge(startpoint, endpoint);
            edges_.push_back(new_edge);

            // Link the edge with the cell
            world_[cellIdx(i,j)].wall[S] = true;
            world_[cellIdx(i,j)].wall_id[S] = edges_.size()-1;
          }
        }

        // Eastern edge
        if(i+1<world_w_ && world_[cellIdx(i+1,j)].enabled){
          // Eastern cell is occupied. No need for an eastern edge
        }else{
          // Try to extend the eastern edge of the northern neighbour if there
          // is one.
          if((j-1)>=0
          && world_[cellIdx(i,j-1)].enabled
          && world_[cellIdx(i,j-1)].wall[E]){
            int edges_idx = world_[cellIdx(i,j-1)].wall_id[E];
            auto endpoint = edges_[edges_idx].pt2();

            // Extend the endpoint down by 1 cell
            edges_[edges_idx] = cg::Line2d(
                edges_[edges_idx].pt1(), // Original edge start point
                cg::Point2d(endpoint.x(), endpoint.y()+cell_size_));

            world_[cellIdx(i,j)].wall[E] = true;
            world_[cellIdx(i,j)].wall_id[E] = edges_idx;
          }else{
            // No occupied northern neighbour cell. Start a new edge
            cg::Point2d startpoint(i*cell_size_+cell_size_,j*cell_size_);
            cg::Point2d endpoint(i*cell_size_+cell_size_,
                                 j*cell_size_+cell_size_);
            cg::Line2d new_edge(startpoint, endpoint);
            edges_.push_back(new_edge);

            // Link the edge with the cell
            world_[cellIdx(i,j)].wall[E] = true;
            world_[cellIdx(i,j)].wall_id[E] = edges_.size()-1;
          }
        }
      }
    }
  }

  int cellIdx(int x, int y){return y*world_w_+x;}

};

int main(){
  LineOfSight engine;
  if(!engine.Construct(320, 240, 10,10)) return 0;
  engine.Start();
  return 0;
}
