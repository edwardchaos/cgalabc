#define OLC_PGE_APPLICATION

#include <vector>
#include <unordered_set>

#include "olcPixelGameEngine.h"
#include "include/Algo2d.h"

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
    convertWorldCellsToEdges();

    // Right clicking shows the light and shadows
    if(GetMouse(1).bHeld){
      auto rays = std::move(computeRaySegments());
      auto collisions = std::move(computeClosestRayCollision(rays));
      auto sorted_collision_pts = std::move(sortCollisionsByAngle(collisions));
      drawVisibleArea(sorted_collision_pts);
    }

    drawWorld();
    return true;
  }

 private:
  int cell_size_ = 10;
  int world_w_, world_h_;
  std::vector<sCell> world_;
  std::vector<cg::Line2d> edges_;

  std::vector<cg::Line2d> computeRaySegments(){
    std::vector<cg::Line2d> rays;
    std::unordered_set<cg::Point2d, cg::hashPoint2d> ray_casted_pts;
    auto mouse_x = GetMouseX();
    auto mouse_y = GetMouseY();
    cg::Point2d mouse_pos(mouse_x, mouse_y);
    double dtheta = 0.005; // A little perturbation from each vertex
    double cos_t = cos(dtheta);
    double sin_t = sin(dtheta);

    for(const auto & edge : edges_){
      if(ray_casted_pts.find(edge.pt1()) == ray_casted_pts.end()
      && mouse_pos != edge.pt1()){
        cg::Line2d ray(mouse_pos, edge.pt1());
        rays.emplace_back(mouse_pos, edge.pt1());
        ray_casted_pts.insert(edge.pt1());

        double dx1,dx2,dy1,dy2;
        dx1 = ray.vec().x()*cos_t - ray.vec().y()*sin_t;
        dy1 = ray.vec().x()*sin_t + ray.vec().y()*cos_t;
        dx2 = ray.vec().x()*cos_t + ray.vec().y()*sin_t;
        dy2 = -ray.vec().x()*sin_t + ray.vec().y()*cos_t;

        // Rays are too short to reach the back wall, extend them out to
        // "infinite" length
        auto perturb_angle_pt1 = mouse_pos + cg::Point2d(dx1,dy1)*10000;
        auto perturb_angle_pt2 = mouse_pos + cg::Point2d(dx2,dy2)*10000;
        rays.emplace_back(mouse_pos, perturb_angle_pt1);
        rays.emplace_back(mouse_pos, perturb_angle_pt2);
      }

      if(ray_casted_pts.find(edge.pt2()) == ray_casted_pts.end()
      && mouse_pos != edge.pt2()){
        cg::Line2d ray(mouse_pos, edge.pt2());
        rays.emplace_back(mouse_pos, edge.pt2());
        ray_casted_pts.insert(edge.pt2());

        double dx1,dx2,dy1,dy2;
        dx1 = ray.vec().x()*cos_t - ray.vec().y()*sin_t;
        dy1 = ray.vec().x()*sin_t + ray.vec().y()*cos_t;
        dx2 = ray.vec().x()*cos_t + ray.vec().y()*sin_t;
        dy2 = -ray.vec().x()*sin_t + ray.vec().y()*cos_t;

        auto perturb_angle_pt1 = mouse_pos + cg::Point2d(dx1,dy1)*10000;
        auto perturb_angle_pt2 = mouse_pos + cg::Point2d(dx2,dy2)*10000;
        rays.emplace_back(mouse_pos, perturb_angle_pt1);
        rays.emplace_back(mouse_pos, perturb_angle_pt2);
      }
    }
    return rays;
  }

  std::unordered_set<cg::Point2d, cg::hashPoint2d>
  computeClosestRayCollision(const std::vector<cg::Line2d> & rays){
    std::unordered_set<cg::Point2d, cg::hashPoint2d> collisions;

    // Compare ray with all edges to check for collisions.
    for(const auto & ray : rays){
      // Find the closest intersect point on all edges for this ray
      double min_dist = 1e9;
      cg::Point2d min_point;
      cg::Point2d mouse_pos(GetMouseX(), GetMouseY());
      for(const auto & edge : edges_){
        if(!cg::intersects(ray, edge)) continue;

        auto int_point = cg::intersectPoint(ray,edge);
        if(!int_point) continue;
        double dist = mouse_pos.dist(*int_point);
        if(dist < min_dist){
          min_dist = dist;
          min_point = *int_point;
        }
      }

      collisions.insert(min_point);
    }

    return collisions;
  }

  std::vector<cg::Point2d>
  sortCollisionsByAngle(
      std::unordered_set<cg::Point2d,cg::hashPoint2d> &collisions){
    std::vector<cg::Point2d> sorted_pts;
    sorted_pts.reserve(collisions.size());

    // change unordered set to vector
    for(auto it = collisions.begin(); it != collisions.end();){
      sorted_pts.push_back(collisions.extract(it++).value());
    }

    cg::Point2d(GetMouseX(), GetMouseY());
    auto angle_sort = [&](const cg::Point2d &pt1, const cg::Point2d &pt2){
      double dx1, dx2, dy1, dy2;

      dx1 = pt1.x() - GetMouseX();
      dy1 = pt1.y() - GetMouseY();
      dx2 = pt2.x() - GetMouseX();
      dy2 = pt2.y() - GetMouseY();

      return atan2(dy1, dx1) < atan2(dy2, dx2);
    };

    std::sort(sorted_pts.begin(), sorted_pts.end(), angle_sort);
    return sorted_pts;
  }

  void drawCollisionPoints(std::vector<cg::Point2d> &sorted_collision_pts){
    for(const auto &pt : sorted_collision_pts){
      DrawCircle(pt.x(), pt.y(), 3, olc::RED);
    }
  }

  void drawVisibleArea(std::vector<cg::Point2d> &sorted_pts){
    if(sorted_pts.size() < 2) return;

    for(int i = 0; i < sorted_pts.size()-1; ++i){
      FillTriangle(sorted_pts[i].x(), sorted_pts[i].y(),
                   GetMouseX(), GetMouseY(),
                   sorted_pts[i+1].x(), sorted_pts[i+1].y());
    }
    // triangle from last to first point
    FillTriangle(sorted_pts.back().x(), sorted_pts.back().y(),
                 GetMouseX(), GetMouseY(),
                 sorted_pts.front().x(), sorted_pts.front().y());
  }

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
  if(!engine.Construct(1920, 1080, 1,1)) return 0;
  engine.Start();
  return 0;
}
