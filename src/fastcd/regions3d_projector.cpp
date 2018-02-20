// Copyright 2017 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de)
#include "fastcd/regions3d_projector.h"
#include <map>
#include <set>
#include <utility>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "fastcd/point_covariance2d.h"

namespace fastcd {

Regions3dProjector::Regions3dProjector(
    std::shared_ptr<ImageSequence> img_sequence, int threshold_change_area,
    double chi_square2d, double chi_square3d) {
  img_sequence->ComputeAndMatchRegions(threshold_change_area);
  
  std::map<int, std::set<int>> labels;
  for (size_t i = 0; i < img_sequence->size(); i++) {
    std::vector<int> l = (*img_sequence)[i].ComputeMeansCovariances(chi_square2d);
    for (int idx : l) {
      if (labels.count(idx) > 0) {
        labels[idx].insert(i);
      } else {
        std::set<int> tmp;
        tmp.insert(i);
        labels.insert(std::pair<int, std::set<int>>(idx, tmp));
      }
    }
  }
  for (auto l : labels) {
    if (l.second.size() > 1) {
      // Point triangulation (forward intersection)
      Eigen::Vector3d triangulated_mean = TriangulateMeanRay(img_sequence, l);
      std::vector<Eigen::Vector3d> projected_vertices =
          ProjectVertices(img_sequence, triangulated_mean, l);
      Eigen::Vector3d mean = ComputeMean(projected_vertices);
      Eigen::Matrix3d covariance = ComputeCovariance(projected_vertices, mean);
      PointCovariance3d cov(mean, covariance, chi_square3d);
      projected_regions_.push_back(cov);
    }
  }
}

Eigen::Vector3d Regions3dProjector::TriangulateMeanRay(
    const std::shared_ptr<ImageSequence> &img_sequence,
    const std::pair<int, std::set<int>> &label) {
  Eigen::Matrix<double, Eigen::Dynamic, 4> A;
  int size = 0;
  for (uint32_t i = 0; i < img_sequence->size(); i++) {
    if (label.second.find(i) != label.second.end()) {
      size++;
    }
  }
  A.resize(size * 3, 4);
  Eigen::Vector3d point, ray;
  int idx = 0;
  for (uint32_t i = 0; i < img_sequence->size(); i++) {
    if (label.second.find(i) != label.second.end()) {
      PointCovariance2d point_covariance =
          (*img_sequence)[i].GetMeansCovariances()[label.first];
      point << point_covariance.Point()(0), point_covariance.Point()(1), 1;
      ray = (*img_sequence)[i].GetCamera().GetInvK() * point;
      Eigen::Matrix3d skew_ray = SkewSymmetric3d(ray);
      Eigen::Matrix<double, 3, 4> P;
      P.block<3, 3>(0, 0) =
          (*img_sequence)[i].GetCamera().GetPose().block<3, 3>(0, 0).transpose();
      P.block<3, 1>(0, 3) =
          -P.block<3, 3>(0, 0) *
          (*img_sequence)[i].GetCamera().GetPose().block<3, 1>(0, 3);
      A.block<3, 4>(idx++ * 3, 0) = skew_ray * P;
    }
  }
  Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 4>> svd(
      A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Vector4d triangulated_mean =
      svd.matrixV().block<4, 1>(0, 3).transpose() / svd.matrixV()(3, 3);
  return triangulated_mean.block<3, 1>(0, 0);
}

std::vector<Eigen::Vector3d> Regions3dProjector::ProjectVertices(
    const std::shared_ptr<ImageSequence> &img_sequence,
    const Eigen::Vector3d &triangulated_mean,
    const std::pair<int, std::set<int>> &label) {
  std::vector<Eigen::Vector3d> vertices3d;
  for (uint32_t i = 0; i < img_sequence->size(); i++) {
    if (label.second.find(i) != label.second.end()) {
      PointCovariance2d point_covariance =
          (*img_sequence)[i].GetMeansCovariances()[label.first];
      // Distance camera/triangulated_mean
      Eigen::Vector3d hom_mean;
      hom_mean << point_covariance.Point()(0), point_covariance.Point()(1), 1;
      Eigen::Vector3d mean_ray =
          ((*img_sequence)[i].GetCamera().GetPose().block<3, 3>(0, 0) *
           (*img_sequence)[i].GetCamera().GetInvK() * hom_mean)
              .normalized();
      double distance = mean_ray.transpose() * triangulated_mean;
      // Plane on 3d point with normal direction = ray direction
      Eigen::Vector4d plane;
      plane << mean_ray(0), mean_ray(1), mean_ray(2), -distance;
      for (auto vertex : point_covariance.Vertices()) {
        // Convert to Pluecker line
        Eigen::Vector3d hom_vertex;
        hom_vertex << vertex(0), vertex(1), 1;
        Eigen::Vector3d ray =
            ((*img_sequence)[i].GetCamera().GetPose().block<3, 3>(0, 0) *
             (*img_sequence)[i].GetCamera().GetInvK() * hom_vertex)
                .normalized();
        Eigen::Matrix<double, 6, 1> pluecker_line;
        // The line passes through 2 points
        Eigen::Vector3d p1 = (*img_sequence)[i].GetCamera().GetPosition();
        Eigen::Vector3d p2 = (*img_sequence)[i].GetCamera().GetPosition() + ray;
        pluecker_line.block<3, 1>(0, 0) = ray;
        pluecker_line.block<3, 1>(3, 0) = p1.cross(p2);
        // Intersection line/plane
        Eigen::Matrix3d skew_pluecker =
            SkewSymmetric3d(pluecker_line.block<3, 1>(3, 0));
        Eigen::Matrix4d pluecker_mat;  // transposed Pluecker matrix
        pluecker_mat.block<3, 3>(0, 0) = skew_pluecker;
        pluecker_mat.block<3, 1>(0, 3) = pluecker_line.block<3, 1>(0, 0);
        pluecker_mat.block<1, 3>(3, 0) =
            -pluecker_line.block<3, 1>(0, 0).transpose();
        pluecker_mat(3, 3) = 0;
        Eigen::Vector4d vertex_hom = pluecker_mat * plane;
        Eigen::Vector3d vertex3d = vertex_hom.block<3, 1>(0, 0) / vertex_hom(3);
        vertices3d.push_back(vertex3d);
      }
    }
  }
  return vertices3d;
}

Eigen::Matrix3d Regions3dProjector::SkewSymmetric3d(
    const Eigen::Vector3d &vec) {
  Eigen::Matrix3d skew_symmetric;
  skew_symmetric << 0, -vec(2), vec(1),
                    vec(2), 0, -vec(0),
                    -vec(1), vec(0), 0;
  return skew_symmetric;
}

Eigen::Vector3d Regions3dProjector::ComputeMean(
    const std::vector<Eigen::Vector3d> &points) {
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (auto point : points) mean += point;
  mean /= points.size();
  return mean;
}

Eigen::Matrix3d Regions3dProjector::ComputeCovariance(
    const std::vector<Eigen::Vector3d> &points_list,
    const Eigen::Vector3d &mean) {
  Eigen::Matrix<double, Eigen::Dynamic, 3> points;
  points.resize(points_list.size(), 3);
  int i = 0;
  for (auto point : points_list) {
    points.block<1, 3>(i++, 0) = (point-mean).transpose();
  }
  Eigen::Matrix3d covariance;
  covariance = points.transpose() * points;
  covariance /= points_list.size();
  return covariance;
}

std::vector<PointCovariance3d> Regions3dProjector::GetProjectedRegions() {
  return projected_regions_;
}

}  // namespace fastcd
