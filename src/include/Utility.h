#pragma once

#include <string>
#include <sstream>
#include <fstream>

#ifdef Success
  #undef Success
#endif
#include <Eigen/Dense>

#include "Mesh.h"

using Eigen::Vector3d;
namespace cg{

/*
 * Create mesh from obj file
 * Triangle face vertices need to be counter clockwise ordered.
 */
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
  };

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
} // namespace cg
