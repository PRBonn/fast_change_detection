// Copyright 2017 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de)
#include "fastcd/point_covariance2d.h"
#include <Eigen/Eigenvalues>

namespace fastcd {

PointCovariance2d::PointCovariance2d() {}

PointCovariance2d::PointCovariance2d(const Eigen::Vector2d &point,
                       const Eigen::Matrix2d &covariance, double chi_square)
    : point_(point), covariance_(covariance) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(covariance_);
  eigenvalues_ = es.eigenvalues();
  eigenvectors_ = es.eigenvectors();
  angle_ = atan2(eigenvectors_(1, 1), eigenvectors_(1, 0));
  if (angle_ < 0) angle_ += 2 * M_PI;
  half_major_axis_size_ = sqrt(chi_square) * sqrt(eigenvalues_(1));
  half_minor_axis_size_ = sqrt(chi_square) * sqrt(eigenvalues_(0));
  Eigen::Matrix2d R;
  R << cos(angle_), -sin(angle_), sin(angle_), cos(angle_);
  Eigen::Vector2d xaxis, yaxis, vertex;
  yaxis << 0, 1;
  xaxis << 1, 0;
  vertex = R * half_major_axis_size_ * xaxis + point;
  vertices_.push_back(vertex);
  vertex = -R * half_major_axis_size_ * xaxis + point;
  vertices_.push_back(vertex);
  vertex = R * half_minor_axis_size_ * yaxis + point;
  vertices_.push_back(vertex);
  vertex = -R * half_minor_axis_size_ * yaxis + point;
  vertices_.push_back(vertex);
}

Eigen::Vector2d PointCovariance2d::Point() { return point_; }

Eigen::Matrix2d PointCovariance2d::Covariance() { return covariance_; }

Eigen::Vector2d PointCovariance2d::Eigenvalues() { return eigenvalues_; }

Eigen::Matrix2d PointCovariance2d::Eigenvectors() { return eigenvectors_; }

double PointCovariance2d::Angle() { return angle_; }

double PointCovariance2d::HalfMajorAxisSize() { return half_major_axis_size_; }

double PointCovariance2d::HalfMinorAxisSize() { return half_minor_axis_size_; }

std::vector<Eigen::Vector2d> PointCovariance2d::Vertices() { return vertices_; }

}  // namespace fastcd
