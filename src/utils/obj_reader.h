// Copyright 2017 Jens Behley (jens.behley@igg.uni-bonn.de), Cyrill Stachniss, University of Bonn
#pragma once

#include <map>
#include "fastcd/mesh.h"

/**
 * @brief      Class for loading a mesh from a Wavefront .obj file.
 */
class ObjReader {
 public:
  /**
   * @brief      Read a mesh from a file with the given filename.
   *
   * @param[in]  filename  The filename
   *
   * @return     The mesh.
   */
  static Mesh FromFile(const std::string& filename);

 protected:
  /**
   * @brief      Parse the given material file
   *
   * @param[in]  filename   The filename
   * @param[out] materials  The materials
   */
  static void ParseMaterials(const std::string& filename,
                             std::map<std::string, Mesh::Material>& materials);
};
