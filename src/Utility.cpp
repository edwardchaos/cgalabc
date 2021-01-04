#include "Utility.h"

#include <algorithm>
#include "Point2d.h"

namespace cg{

std::string getResourcesPath(){
  std::string file_path = __FILE__;
  std::string dir_path = file_path.substr(0, file_path.rfind("/"));
  dir_path += "/../resources/";
  return dir_path;
}

Material_ptr defaultMaterial(){
  Material_ptr default_mat = std::make_shared<Material>();
  std::shared_ptr<olc::Sprite> sprite=std::make_shared<olc::Sprite>();
  if(sprite->loader==nullptr) return default_mat;

  auto tex_path = getResourcesPath() + "sample_textures/rainbow.png";

  sprite->LoadFromFile(tex_path);
  default_mat->texture = sprite;
  return default_mat;
}

Mesh_ptr loadOBJ(const std::string& path_to_obj, bool ccw_points){
  std::ifstream obj_file(path_to_obj);
  if(!obj_file.is_open()) return nullptr;

  std::vector<Vector3d> vertices;
  std::vector<Vector2d> texture_vertices;
  std::vector<Vector3d> vertex_normals;
  Mesh_ptr mesh = std::make_shared<Mesh>();

  std::string line;
  while(getline(obj_file,line)){
    std::istringstream iss(line);

    while(iss){
      std::string subs;
      iss >> subs;

      if("v" == subs) {
        std::string x,y,z;

        iss >> x;
        iss >> y;
        iss >> z;

        vertices.emplace_back(std::stod(x),std::stod(y),std::stod(z));
      }
      else if("vt" == subs){
        std::string x,y,junk;

        iss >> x;
        iss >> y;
        iss >> junk;

        texture_vertices.emplace_back(std::stod(x),std::stod(y));
      }
      else if("vn" == subs){
        std::string x,y,z;

        iss >> x;
        iss >> y;
        iss >> z;
        Vector3d v_norm(std::stod(x),std::stod(y),std::stod(z));
        v_norm.normalize();
        vertex_normals.emplace_back(v_norm);
      }
      else if("f" == subs) {
        std::string i1, i2, i3;
        if(ccw_points) {
          iss >> i1;
          iss >> i2;
          iss >> i3;
        }else{
          iss >> i1;
          iss >> i3;
          iss >> i2;
        }

        // Counter number of slashes
        auto num_slashes = std::count(i1.begin(),i1.end(),'/');
        if(num_slashes == 0){
          // Simplest case, each string is just a vertex index
          int ii1,ii2,ii3;

          ii1 = std::stoi(i1);
          if(ii1 > 0) --ii1;
          else ii1+=vertices.size();

          ii2 = std::stoi(i2);
          if(ii2 > 0) --ii2;
          else ii2+=vertices.size();

          ii3 = std::stoi(i3);
          if(ii3 > 0) --ii3;
          else ii3+=vertices.size();

          mesh->tris.emplace_back(vertices[ii1],
                                  vertices[ii2],
                                  vertices[ii3]);
        }else if(num_slashes == 1){
          // [0] is vertex idx. [1] is texture vertex
          int int_idx1[2];
          std::string idx1;
          std::stringstream ss1(i1);
          int int_idx2[2];
          std::string idx2;
          std::stringstream ss2(i2);
          int int_idx3[2];
          std::string idx3;
          std::stringstream ss3(i3);

          for(int i = 0; i < 2; ++i){
            std::getline(ss1, idx1, '/');
            int_idx1[i] = std::stoi(idx1);
            if(int_idx1[i] > 0) --int_idx1[i]; // change to 0 indexed
            std::getline(ss2, idx2, '/');
            int_idx2[i] = std::stoi(idx2);
            if(int_idx2[i] > 0) --int_idx2[i]; // change to 0 indexed
            std::getline(ss3, idx3, '/');
            int_idx3[i] = std::stoi(idx3);
            if(int_idx3[i] > 0) --int_idx3[i]; // change to 0 indexed
            if(i==0){ // Vertex index
              if(int_idx1[i] < 0) int_idx1[i]+=vertices.size();
              if(int_idx2[i] < 0) int_idx2[i]+=vertices.size();
              if(int_idx3[i] < 0) int_idx3[i]+=vertices.size();
            }else{ // Texture index
              if(int_idx1[i] < 0) int_idx1[i]+=texture_vertices.size();
              if(int_idx2[i] < 0) int_idx2[i]+=texture_vertices.size();
              if(int_idx3[i] < 0) int_idx3[i]+=texture_vertices.size();
            }
          }

          mesh->tris.emplace_back(vertices[int_idx1[0]],
                                  vertices[int_idx2[0]],
                                  vertices[int_idx3[0]],
                                  texture_vertices[int_idx1[1]],
                                  texture_vertices[int_idx2[1]],
                                  texture_vertices[int_idx3[1]]);
        }else if(num_slashes == 2){
          // Similar thing as above, but now with normal as well.
          // [0] is vertex idx. [1] is texture idx. [2] is normal idx
          int int_idx1[3];
          std::string idx1;
          std::stringstream ss1(i1);
          int int_idx2[3];
          std::string idx2;
          std::stringstream ss2(i2);
          int int_idx3[3];
          std::string idx3;
          std::stringstream ss3(i3);

          for(int i = 0; i < 3; ++i){
            std::getline(ss1, idx1, '/');
            int_idx1[i] = std::stoi(idx1);
            if(int_idx1[i] > 0) --int_idx1[i]; // change to 0 indexed
            std::getline(ss2, idx2, '/');
            int_idx2[i] = std::stoi(idx2);
            if(int_idx2[i] > 0) --int_idx2[i]; // change to 0 indexed
            std::getline(ss3, idx3, '/');
            int_idx3[i] = std::stoi(idx3);
            if(int_idx3[i] > 0) --int_idx3[i]; // change to 0 indexed
            if(i==0){ // Vertex index
              if(int_idx1[i] < 0) int_idx1[i]+=vertices.size();
              if(int_idx2[i] < 0) int_idx2[i]+=vertices.size();
              if(int_idx3[i] < 0) int_idx3[i]+=vertices.size();
            }else if(i==1){ // Texture index
              if(int_idx1[i] < 0) int_idx1[i]+=texture_vertices.size();
              if(int_idx2[i] < 0) int_idx2[i]+=texture_vertices.size();
              if(int_idx3[i] < 0) int_idx3[i]+=texture_vertices.size();
            }else{
              if(int_idx1[i] < 0) int_idx1[i]+=vertex_normals.size();
              if(int_idx2[i] < 0) int_idx2[i]+=vertex_normals.size();
              if(int_idx3[i] < 0) int_idx3[i]+=vertex_normals.size();
            }
          }

          // Currently doesn't support creating triangles given vertex normals.
          // It just ignores the normal information.
          mesh->tris.emplace_back(vertices[int_idx1[0]],
                                  vertices[int_idx2[0]],
                                  vertices[int_idx3[0]],
                                  texture_vertices[int_idx1[1]],
                                  texture_vertices[int_idx2[1]],
                                  texture_vertices[int_idx3[1]]);
        }else{
          throw std::invalid_argument("OBJ faces can only have 0,1, or 2 '/'.");
        }

      }
    }
  }
  obj_file.close();

  return mesh;
}

Mesh_ptr cube(){
  Mesh_ptr cube = std::make_shared<Mesh>();

  // create cube
  // South face
  cube->tris.emplace_back(Vector3d(0,0,0),Vector3d(-1,1,0),Vector3d(0,1,0),
                          Vector2d(0,1),Vector2d(1,0),Vector2d(0,0));
  cube->tris.emplace_back(Vector3d(0,0,0),Vector3d(-1,0,0),Vector3d(-1,1,0),
                          Vector2d(0,1),Vector2d(1,1),Vector2d(1,0));

  // East face
  cube->tris.emplace_back(Vector3d(-1,0,0),Vector3d(-1,1,1),Vector3d(-1,1,0),
                          Vector2d(0,1),Vector2d(1,0),Vector2d(0,0));
  cube->tris.emplace_back(Vector3d(-1,0,0),Vector3d(-1,0,1),Vector3d(-1,1,1),
                          Vector2d(0,1),Vector2d(1,1),Vector2d(1,0));

  // West face
  cube->tris.emplace_back(Vector3d(0,0,0),Vector3d(0,1,0),Vector3d(0,1,1),
                          Vector2d(1,1),Vector2d(1,0),Vector2d(0,0));
  cube->tris.emplace_back(Vector3d(0,0,0),Vector3d(0,1,1),Vector3d(0,0,1),
                          Vector2d(1,1),Vector2d(0,0),Vector2d(0,1));

  // North face
  cube->tris.emplace_back(Vector3d(0,0,1),Vector3d(0,1,1),Vector3d(-1,1,1),
                          Vector2d(1,1),Vector2d(1,0),Vector2d(0,0));
  cube->tris.emplace_back(Vector3d(0,0,1),Vector3d(-1,1,1),Vector3d(-1,0,1),
                          Vector2d(1,1),Vector2d(0,0),Vector2d(0,1));

  // Top face
  cube->tris.emplace_back(Vector3d(0,1,0),Vector3d(-1,1,1),Vector3d(0,1,1),
                          Vector2d(0,1),Vector2d(1,0),Vector2d(0,0));
  cube->tris.emplace_back(Vector3d(0,1,0),Vector3d(-1,1,0),Vector3d(-1,1,1),
                          Vector2d(0,1),Vector2d(1,1),Vector2d(1,0));

  // Bottom face
  cube->tris.emplace_back(Vector3d(0,0,0),Vector3d(0,0,1),Vector3d(-1,0,1),
                          Vector2d(0,0),Vector2d(0,1),Vector2d(1,1));
  cube->tris.emplace_back(Vector3d(0,0,0),Vector3d(-1,0,1),Vector3d(-1,0,0),
                          Vector2d(0,0),Vector2d(1,1),Vector2d(1,0));

  auto material = defaultMaterial();
  for(auto&tri:cube->tris) {
    tri.material = material;
  }

  return cube;
}

Mesh_ptr teapot(){
  auto path = getResourcesPath()+"/sample_models/teapot.obj";
  auto teapot = cg::loadOBJ(path, true);
  auto default_material = defaultMaterial();
  for(auto &tri : teapot->tris)
    tri.material = default_material;

  return teapot;
}

Mesh_ptr simpleTriangle(){
  auto default_material = defaultMaterial();
  Triangle triangle{
      Vector3d(0,-1,10),{-1,-1,10},{0,1,10},
      Vector2d(0,1),{1,1},{0,0}};
  triangle.material = default_material;

  Mesh_ptr tri_mesh = std::make_shared<Mesh>();
  tri_mesh->tris.push_back(triangle);
  return tri_mesh;
}

Mesh_ptr thinTriangles(){
  auto default_material = defaultMaterial();
  cg::Triangle thin_bottom{
    Vector3d(0,0,10),
    Vector3d(-10,0,10),
    Vector3d(-10,0.5,10)};
  thin_bottom.material = default_material;

  cg::Triangle thin_top{
    Vector3d(0,0,10),
    Vector3d(-10,0.5,10),
    Vector3d(0,0.5,10)};
  thin_top.material = default_material;

  cg::Mesh_ptr thin_tri_mesh = std::make_shared<Mesh>();
  thin_tri_mesh->tris.push_back(thin_bottom);
  thin_tri_mesh->tris.push_back(thin_top);

  return thin_tri_mesh;
}

Mesh_ptr worldAxis(){
  auto default_material = defaultMaterial();
  auto path = getResourcesPath()+"/sample_models/axis.obj";
  auto axis= cg::loadOBJ(path, true);
  for(auto&tri: axis->tris)
    tri.material = default_material;
  return axis;
}

Mesh_ptr teddy(){
  auto default_material = defaultMaterial();
  auto path = getResourcesPath()+"/sample_models/teddy.obj";
  auto teddy = cg::loadOBJ(path, true);
  for(auto&tri: teddy->tris)
    tri.material = default_material;
  return teddy;
}

Mesh_ptr spyro(){
  auto default_material = defaultMaterial();
  auto path = getResourcesPath()+"/sample_models/spyro.obj";
  auto spyro = cg::loadOBJ(path, true);
  for(auto&tri: spyro->tris)
    tri.material = default_material;
  return spyro;
}

Eigen::Matrix4d rotateX(double theta){
  Eigen::Matrix4d rotX = Eigen::Matrix4d::Identity();
  rotX(1,1) = cos(theta);
  rotX(1,2) = -sin(theta);
  rotX(2,1) = sin(theta);
  rotX(2,2) = cos(theta);

  return rotX;
}

Eigen::Matrix4d rotateY(double theta){
  Eigen::Matrix4d rotY = Eigen::Matrix4d::Identity();
  rotY(0,0) = cos(theta);
  rotY(0,2) = sin(theta);
  rotY(2,0) = -sin(theta);
  rotY(2,2) = cos(theta);

  return rotY;
}

Eigen::Matrix4d rotateZ(double theta){
  Eigen::Matrix4d rotZ = Eigen::Matrix4d::Identity();
  rotZ(0,0) = cos(theta);
  rotZ(0,1) = -sin(theta);
  rotZ(1,0) = sin(theta);
  rotZ(1,1) = cos(theta);

  return rotZ;
}

Eigen::Matrix4d translation(double dx, double dy, double dz){
  Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
  translation(0,3) = dx;
  translation(1,3) = dy;
  translation(2,3) = dz;

  return translation;
}

Eigen::Vector4d transformPoint(const Eigen::Vector4d &pt,
                               const Eigen::Matrix4d &tf){
 return tf*pt;
}

Triangle transformTriangle(const Triangle& tri, const Matrix4d& tf){
  Triangle transformed_tri;
  for(int i = 0; i < 3; ++i){
    transformed_tri.points[i] = std::move(transformPoint(tri.points[i],tf));
    transformed_tri.t[i] = Vector3d(tri.t[i]); // Transform doesn't affect

    // Vertex normals only define direction.
    transformed_tri.vertex_normals[i] =
        tf.block(0,0,3,3)*tri.vertex_normals[i];
  }
  transformed_tri.material = tri.material;
  return transformed_tri;
}

std::shared_ptr<Vector3d> planeLineIntersect(
    const Vector3d &pt1, const Vector3d &pt2,
    const Vector3d &plane_unit_normal, const Vector3d &pt_on_plane, double&t){

  // Ensure it's actually a unit vector
  Vector3d plane_norm(plane_unit_normal);
  plane_norm.normalize();

  double d1 = plane_norm.dot(pt1-pt_on_plane);
  double d2 = plane_norm.dot(pt2-pt_on_plane);

  // Line and plane are parallel
  if(fabs(d1-d2) < EPS) return nullptr;

  t = d1/(d1-d2);

  // Line segment isn't long enough to intersect plane.
  if(t < -EPS || t > 1+EPS) return nullptr;

  std::shared_ptr<Vector3d> intersect_pt = std::make_shared<Vector3d>();
  *intersect_pt = pt1 + (pt2-pt1)*t;

  return intersect_pt;
}

std::shared_ptr<Vector2d> lineLineIntersect2d(
    const Vector2d &pt1, const Vector2d &pt2,
    const Vector2d &line_unit_normal, const Vector2d &pt_on_line, double&t){

  // Ensure it's actually a unit vector
  Vector2d line_norm(line_unit_normal);
  line_norm.normalize();

  double d1 = line_norm.dot(pt1-pt_on_line);
  double d2 = line_norm.dot(pt2-pt_on_line);

  // Lines are parallel
  if(fabs(d1-d2) < EPS) return nullptr;

  t = d1/(d1-d2);

  // Line segment isn't long enough to intersect.
  if(t < -EPS || t > 1+EPS) return nullptr;

  std::shared_ptr<Vector2d> intersect_pt = std::make_shared<Vector2d>();
  *intersect_pt = pt1 + (pt2-pt1)*t;

  return intersect_pt;
}

Vector3d slerp(const Vector3d &from, const Vector3d &to, double s){
  if(from.isApprox(to))return from;
  double theta = acos(from.dot(to));
  double sin_theta, sin_1_s_theta, sin_s_theta;
  double stheta = s*theta;
  if(theta < 0.244){
    sin_theta = theta;
    sin_1_s_theta = theta - stheta;
    sin_s_theta = stheta;
  }
  else{
    sin_theta = sin(theta);
    sin_1_s_theta = sin(theta - stheta);
    sin_s_theta = sin(stheta);
  }

  double alpha = sin_1_s_theta/sin_theta;
  double beta = sin_s_theta/sin_theta;
  return alpha*from+beta*to;
}
} // namespace cg
