// Copyright 2017 Jens Behley (jens.behley@igg.uni-bonn.de), Cyrill Stachniss, University of Bonn
#include "fastcd/mesh.h"

#include <glow/ScopedBinder.h>

namespace glow {
template <>
void GlUniform<Mesh::Material>::bind(GLuint program_id) const {
  GLint loc;
  loc = glGetUniformLocation(program_id, (name_ + ".ambient").c_str());
  glUniform3fv(loc, 1, &data_.ambient.x);
  loc = glGetUniformLocation(program_id, (name_ + ".diffuse").c_str());
  glUniform3fv(loc, 1, &data_.diffuse.x);
  loc = glGetUniformLocation(program_id, (name_ + ".specular").c_str());
  glUniform3fv(loc, 1, &data_.specular.x);
  loc = glGetUniformLocation(program_id, (name_ + ".emission").c_str());
  glUniform3fv(loc, 1, &data_.emission.x);
  loc = glGetUniformLocation(program_id, (name_ + ".shininess").c_str());
  glUniform1f(loc, data_.shininess);
  loc = glGetUniformLocation(program_id, (name_ + ".alpha").c_str());
  glUniform1f(loc, data_.alpha);
}

}  // namespace glow

Mesh::Mesh() {
}

Mesh::Mesh(const std::vector<Vertex>& vertices,
           const std::vector<Triangle>& faces) {
  std::vector<Material> dummy(1);
  initialize(vertices, faces, dummy);
}

Mesh::Mesh(const std::vector<Vertex>& vertices,
           const std::vector<Triangle>& faces,
           const std::vector<Material>& materials) {
  initialize(vertices, faces, materials);
}

float Mesh::length(const glow::vec4& v) {
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Eigen::Vector3f toEigen(const glow::vec4& v) {
  return Eigen::Vector3f(v.x, v.y, v.z);
}

void Mesh::initialize(const std::vector<Vertex>& verts,
                      const std::vector<Triangle>& faces,
                      const std::vector<Material>& materials) {
  if (verts.size() == 0) return;

  std::vector<Vertex> vertices = verts;

  if (std::abs(length(vertices[0].normal) - 1.0f) > 0.01) {
    // assume, no normals given:
    for (auto face : faces) {
      Eigen::Vector3f v0 = toEigen(vertices[face.vertices[0]].position);
      Eigen::Vector3f v1 = toEigen(vertices[face.vertices[1]].position);
      Eigen::Vector3f v2 = toEigen(vertices[face.vertices[2]].position);

      Eigen::Vector3f n = ((v1 - v0).cross(v2 - v0)).normalized();
      for (uint32_t i = 0; i < 3; ++i) {
        vertices[face.vertices[i]].normal.x = n.x();
        vertices[face.vertices[i]].normal.y = n.y();
        vertices[face.vertices[i]].normal.z = n.z();
      }
    }
  }
  std::vector<std::vector<uint32_t> > tris(std::max((int)materials.size(), 1));
  for (uint32_t i = 0; i < faces.size(); ++i) {
    assert(faces[i].material < tris.size());
    tris[faces[i].material].push_back(faces[i].vertices[0]);
    tris[faces[i].material].push_back(faces[i].vertices[1]);
    tris[faces[i].material].push_back(faces[i].vertices[2]);
  }

  // reorder triangles & matrials s.t. transparent materials are drawn last.
  std::vector<Material> reordered_materials(materials.size());
  std::vector<std::vector<uint32_t> > reordered_tris(tris.size());
  uint32_t opaque_idx = 0;
  uint32_t transparent_idx = materials.size() - 1;

  for (uint32_t i = 0; i < tris.size(); ++i) {
    if (materials[i].alpha < 1.) {
      reordered_tris[transparent_idx] = tris[i];
      reordered_materials[transparent_idx] = materials[i];
      tris[i].clear();  // no need to keep data.
      transparent_idx -= 1;
    } else {
      reordered_tris[opaque_idx] = tris[i];
      reordered_materials[opaque_idx] = materials[i];
      tris[i].clear();  // no need to keep data.
      opaque_idx += 1;
    }
  }

  for (uint32_t i = 0; i < reordered_materials.size(); ++i) {
    materials_.push_back(
        glow::GlUniform<Material>("material", reordered_materials[i]));
  }

  // copying the data.
  vertices_.assign(vertices);
  for (uint32_t i = 0; i < reordered_tris.size(); ++i) {
    glow::GlBuffer<uint32_t> buf(glow::BufferTarget::ELEMENT_ARRAY_BUFFER,
                                 glow::BufferUsage::STATIC_DRAW);
    buf.assign(reordered_tris[i]);
    triangles_.push_back(buf);
  }

  // computing boundaries
  max_ << -10000, -10000, -10000;
  min_ << 10000, 10000, 10000;
  for (uint32_t i = 0; i < vertices.size(); ++i) {
    if (vertices[i].position.x > max_(0)) max_(0) = vertices[i].position.x;
    if (vertices[i].position.y > max_(1)) max_(1) = vertices[i].position.y;
    if (vertices[i].position.z > max_(2)) max_(2) = vertices[i].position.z;
    if (vertices[i].position.x < min_(0)) min_(0) = vertices[i].position.x;
    if (vertices[i].position.y < min_(1)) min_(1) = vertices[i].position.y;
    if (vertices[i].position.z < min_(2)) min_(2) = vertices[i].position.z;
  }

  // setting the vaos.
  for (uint32_t i = 0; i < reordered_tris.size(); ++i) {
    vaos_.push_back(glow::GlVertexArray());

    vaos_[i].bind();
    vaos_[i].setVertexAttribute(0, vertices_, 4, glow::AttributeType::FLOAT,
                                false, sizeof(Vertex), (GLvoid*)0);
    vaos_[i].setVertexAttribute(1, vertices_, 4, glow::AttributeType::FLOAT,
                                false, sizeof(Vertex),
                                (GLvoid*)offsetof(Vertex, normal));
    vaos_[i].setVertexAttribute(2, vertices_, 2, glow::AttributeType::FLOAT,
                                false, sizeof(Vertex),
                                (GLvoid*)offsetof(Vertex, texture));
    vaos_[i].enableVertexAttribute(0);
    vaos_[i].enableVertexAttribute(1);
    vaos_[i].enableVertexAttribute(2);
    triangles_[i].bind();
    vaos_[i].release();

    triangles_[i].release();  // release only afterwards

    CheckGlError();
  }
}

void Mesh::draw(glow::GlProgram& program) const {
  bool blending_enabled = false;
  // draw each object for each material.
  for (uint32_t i = 0; i < vaos_.size(); ++i) {
    if (materials_.size() > i) {
      // enable belending for tranparent indexes.
      if (materials_[i].value().alpha < 1. && !blending_enabled) {
        blending_enabled = true;
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA,
                    GL_ONE_MINUS_SRC_ALPHA);  // FIXME: state changed.
        //        glBlendColor(1., 1., 1., materials_[i].value().alpha);
      }
    }

    glow::ScopedBinder<glow::GlVertexArray> vao_bind(vaos_[i]);
    if (materials_.size() > i) program.setUniform(materials_[i]);
    glDrawElements(GL_TRIANGLES, triangles_[i].size(), GL_UNSIGNED_INT, 0);
  }

  if (blending_enabled) glDisable(GL_BLEND);
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> Mesh::GetBoundingBox() {
  return std::pair<Eigen::Vector3d, Eigen::Vector3d>(min_, max_);
}
