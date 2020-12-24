#include "Utility.h"

namespace cg{

Mesh_ptr loadOBJ(std::string path_to_obj){
  std::ifstream obj_file(path_to_obj);
  if(!obj_file.is_open()) return nullptr;

  std::vector<Vector3d> vertices;
  Mesh_ptr mesh = std::make_shared<Mesh>();

  std::string line;
  while(getline(obj_file,line)){
    std::istringstream iss(line);

    while(iss){
      std::string subs;
      iss >> subs;

      if(0 == subs.compare("v")) {
        std::string x,y,z;

        iss >> x;
        iss >> y;
        iss >> z;

        vertices.emplace_back(std::stod(x),std::stod(y),std::stod(z));

      }
      else if(0 == subs.compare("f")) {
        std::string i1, i2, i3;
        iss >> i1;
        iss >> i2;
        iss >> i3;

        mesh->tris.emplace_back(vertices[std::stoi(i1)-1],
                                vertices[std::stoi(i2)-1],
                                vertices[std::stoi(i3)-1]);
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
  cube->tris.emplace_back(Vector3d(0,0,0),Vector3d(1,1,0),Vector3d(0,1,0));
  cube->tris.emplace_back(Vector3d(0,0,0),Vector3d(1,0,0),Vector3d(1,1,0));

  // East face
  cube->tris.emplace_back(Vector3d(1,0,0),Vector3d(1,1,-1),Vector3d(1,1,0));
  cube->tris.emplace_back(Vector3d(1,0,0),Vector3d(1,0,-1),Vector3d(1,1,-1));

  // West face
  cube->tris.emplace_back(Vector3d(0,0,0),Vector3d(0,1,0),Vector3d(0,1,-1));
  cube->tris.emplace_back(Vector3d(0,0,0),Vector3d(0,1,-1),Vector3d(0,0,-1));

  // North face
  cube->tris.emplace_back(Vector3d(0,0,-1),Vector3d(0,1,-1),Vector3d(1,1,-1));
  cube->tris.emplace_back(Vector3d(0,0,-1),Vector3d(1,1,-1),Vector3d(1,0,-1));

  // Top face
  cube->tris.emplace_back(Vector3d(0,1,0),Vector3d(1,1,0),Vector3d(1,1,-1));
  cube->tris.emplace_back(Vector3d(0,1,0),Vector3d(1,1,-1),Vector3d(0,1,-1));

  // Bottom face
  cube->tris.emplace_back(Vector3d(0,0,0),Vector3d(0,0,-1),Vector3d(1,0,-1));
  cube->tris.emplace_back(Vector3d(0,0,0),Vector3d(1,0,-1),Vector3d(1,0,0));

  return cube;
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

Eigen::Vector3d transformPoint(const Eigen::Vector3d &pt,
                               const Eigen::Matrix4d& tf){
  Eigen::Vector4d pt_homo;
  pt_homo.head<3>() = pt;
  pt_homo(3) = 1;
  Eigen::Vector4d pt_tf = tf*pt_homo;
  return {pt_tf(0)/pt_tf(3), pt_tf(1)/pt_tf(3), pt_tf(2)/pt_tf(3)};
}
} // namespace cg