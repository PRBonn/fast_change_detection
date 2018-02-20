// Copyright 2017 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de)
#include "fastcd/point_covariance3d.h"
#include <Eigen/Core>
#include "fastcd/mesh.h"

namespace fastcd {

PointCovariance3d::PointCovariance3d(const Eigen::Vector3d &point,
                           const Eigen::Matrix3d &covariance, double chi_square)
    : point_(point), covariance_(covariance) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance_);
  eigenvalues_ = es.eigenvalues();
  eigenvectors_ = es.eigenvectors();
  scaling_.setZero();
  for (int i = 0; i < 3; i++) {
    scaling_(i, i) = sqrt(chi_square) * sqrt(eigenvalues_(i));
  }
  Eigen::Vector3d vertex;
  vertex << 1, 0, 0;
  vertex = eigenvectors_ * scaling_ * vertex + point_;
  vertices_.push_back(vertex);
  vertex << -1, 0, 0;
  vertex = eigenvectors_ * scaling_ * vertex + point_;
  vertices_.push_back(vertex);
  vertex << 0, 1, 0;
  vertex = eigenvectors_ * scaling_ * vertex + point_;
  vertices_.push_back(vertex);
  vertex << 0, -1, 0;
  vertex = eigenvectors_ * scaling_ * vertex + point_;
  vertices_.push_back(vertex);
  vertex << 0, 0, 1;
  vertex = eigenvectors_ * scaling_ * vertex + point_;
  vertices_.push_back(vertex);
  vertex << 0, 0, -1;
  vertex = eigenvectors_ * scaling_ * vertex + point_;
  vertices_.push_back(vertex);
}

Mesh PointCovariance3d::ToMesh(int stacks, int slices, float r, float g,
                               float b, float a) const {
  Mesh::Material material;
  material.ambient = glow::vec3(0.1, 0.1, 0.1);
  material.diffuse = glow::vec3(r, g, b);
  material.specular = glow::vec3(0, 0, 0);
  material.emission = glow::vec3(0, 0, 0);
  material.alpha = a;
  std::vector<Mesh::Material> materials;
  materials.push_back(material);

  std::vector<Mesh::Vertex> vertices;
  float t_step = M_PI / static_cast<float>(slices);
  float s_step = M_PI / static_cast<float>(stacks);
  for (float t = -M_PI / 2; t <= (M_PI / 2) + .0001; t += t_step) {
    for (float s = -M_PI; s <= M_PI + .0001; s += s_step) {
      Eigen::Vector3d vertex;
      vertex << cos(t) * cos(s), cos(t) * sin(s), sin(t);
      vertex = eigenvectors_ * scaling_ * vertex + point_;
      Mesh::Vertex v1;
      v1.position = glow::vec4(vertex(0), vertex(1), vertex(2), 1);
      vertices.push_back(v1);

      Mesh::Vertex v2;
      vertex << cos(t + t_step) * cos(s), cos(t + t_step) * sin(s),
          sin(t + t_step);
      vertex = eigenvectors_ * scaling_ * vertex + point_;
      v2.position = glow::vec4(vertex(0), vertex(1), vertex(2), 1);
      vertices.push_back(v2);
    }
  }

  std::vector<Mesh::Triangle> triangles;
  for (size_t i = 2; i < vertices.size(); i++) {
    Mesh::Triangle t;
    t.vertices[0] = i-2;
    t.vertices[1] = i % 2 == 0 ? i - 1 : i;
    t.vertices[2] = i % 2 == 0 ? i : i - 1;

    if (t.vertices[0] != t.vertices[1] && t.vertices[1] != t.vertices[2] &&
        t.vertices[2] != t.vertices[0]) {
      triangles.push_back(t);
    }
  }

  Mesh mesh(vertices, triangles, materials);
  return mesh;
}

Eigen::Vector3d PointCovariance3d::Point() const { return point_; }

Eigen::Matrix3d PointCovariance3d::Covariance() const { return covariance_; }

Eigen::Vector3d PointCovariance3d::Eigenvalues() const { return eigenvalues_; }

Eigen::Matrix3d PointCovariance3d::Eigenvectors() const {
  return eigenvectors_;
}

Eigen::Matrix3d PointCovariance3d::Scaling() const { return scaling_; }

std::vector<Eigen::Vector3d> PointCovariance3d::Vertices() const {
  return vertices_;
}

}  // namespace fastcd
