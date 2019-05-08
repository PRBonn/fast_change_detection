// Copyright 2017 Jens Behley (jens.behley@igg.uni-bonn.de), Cyrill Stachniss, University of Bonn
#pragma once

#include <utility>
#include <vector>
#include <glow/GlBuffer.h>
#include <glow/GlProgram.h>
#include <glow/GlVertexArray.h>
#include <glow/glutil.h>

/**
 * @brief      Class that represents a mesh with possibly multiple materials.
 *             It contains all the vertex buffers for drawing.
 */
class Mesh {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief      Struct that represents a material
   */
  struct Material {
   public:
    Material()
        : ambient(0.1, 0.1, 0.1),
          diffuse(0.5, 0.5, 0.8),
          specular(0, 0, 0),
          emission(0, 0, 0) {}

    /** The ambient component of the material */
    glow::vec3 ambient;

    /** The diffuse component of the material */
    glow::vec3 diffuse;

    /** The specular component of the material */
    glow::vec3 specular;

    /** The emission component of the material */
    glow::vec3 emission;

    /** The specular exponent of the material */
    float shininess{1.0f};

    /** The alpha value of the material */
    float alpha{1.0f};
  };


  /**
   * @brief      Struct that represents a vertex
   */
  struct Vertex {
   public:
    /** The position of the vertex */
    glow::vec4 position;

    /** The normal of the vertex */
    glow::vec4 normal;

    /** The texture coordinate of the vertex */
    glow::vec2 texture;
  };

  /**
   * @brief      Struct that represents a triangle
   */
  struct Triangle {
   public:
    /** The indices of the vertices of the triangle */
    uint32_t vertices[3];

    /** The material of the triangle */
    uint32_t material{0};
  };

  /**
   * @brief      Empty constructor.
   */
  Mesh();

  /**
   * @brief      Constructor. Initializes the mesh with a default material.
   *
   * @param[in]  vertices  The vertices
   * @param[in]  faces     The faces
   */
  Mesh(const std::vector<Vertex>& vertices, const std::vector<Triangle>& faces);

  /**
   * @brief      Constructor.
   *
   * @param[in]  vertices   The vertices
   * @param[in]  faces      The faces
   * @param[in]  materials  The materials
   */
  Mesh(const std::vector<Vertex>& vertices, const std::vector<Triangle>& faces,
       const std::vector<Material>& materials);


  /**
   * @brief      Draws the mesh using the specified program.
   *
   * @param      program  The program
   */
  void draw(glow::GlProgram& program) const;

  /**
   * @brief      Gets the material with index idx.
   *
   * @param[in]  idx   The index of the material
   *
   * @return     The material.
   */
  const Material& material(uint32_t idx) const {
    if (idx >= materials_.size())
      throw std::runtime_error("non-existent material.");
    return materials_[idx].value();
  }

  /**
   * @brief      Gets the bounding box of the mesh in the form of a pair of 
   *             points representing the rectangle
   *
   * @return     The bounding box.
   */
  std::pair<Eigen::Vector3d, Eigen::Vector3d> GetBoundingBox();

 protected:
  /**
   * @brief      Initializes the mesh.
   *
   * @param[in]  vertices   The vertices
   * @param[in]  faces      The faces
   * @param[in]  materials  The materials
   */
  void initialize(const std::vector<Vertex>& vertices,
                  const std::vector<Triangle>& faces,
                  const std::vector<Material>& materials);

  /**
   * @brief      Compute the length of a vector.
   *
   * @param[in]  v     The vector
   *
   * @return     The lenght of the vector.
   */
  float length(const glow::vec4& v);

  /** The vertices of the mesh */
  glow::GlBuffer<Vertex> vertices_{glow::BufferTarget::ARRAY_BUFFER,
                                   glow::BufferUsage::STATIC_DRAW};

  /** The VAOs used to draw the mesh */
  std::vector<glow::GlVertexArray> vaos_;

  /** The triangles of the mesh per material */
  std::vector<glow::GlBuffer<uint32_t> > triangles_;

  /** The materials of the mesh */
  std::vector<glow::GlUniform<Material> > materials_;

  /** The vertex of the mesh with the maximum coordinates */
  Eigen::Vector3d max_ = Eigen::Vector3d::Zero();

  /** The vertex of the mesh with the minimum coordinates */
  Eigen::Vector3d min_ = Eigen::Vector3d::Zero();
};
