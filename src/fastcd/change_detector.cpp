// Copyright 2017 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de), Cyrill Stachniss, University of Bonn
#include <utility>
#include <glow/glutil.h>
#include "fastcd/change_detector.h"
#include "fastcd/regions3d_projector.h"
#include "fastcd/processed_image.h"

namespace fastcd {

ChangeDetector::ChangeDetector(const Mesh &mesh,
                               const ChangeDetectorOptions &options)
    : mesh_(mesh), options_(options) {
  image_sequence_ = std::shared_ptr<ImageSequence>(
      new ImageSequence(options_.cache_size, options_.max_comparisons));
}

void ChangeDetector::AddImage(Image &image, int kernel_size) {
  // Scale the image to a fixed resolution
  double scaling = static_cast<double>(options_.rescale_width) /
                   static_cast<double>(image.GetCamera().GetWidth());
  image.Scale(scaling);
  if (!projector_init_) {
    uint32_t width = static_cast<uint32_t>(image.GetCamera().GetWidth());
    uint32_t height = static_cast<uint32_t>(image.GetCamera().GetHeight());
    Eigen::Matrix4f projection = image.GetCamera().GetGlProjection(0.1f, 50.0f);
    depth_projector_ = std::unique_ptr<DepthProjector>(
        new DepthProjector(width, height, projection));
    projector_init_ = true;
  }
  Eigen::Matrix4f view = image.GetCamera().GetGlView();
  depth_projector_->render(mesh_, view);
  std::vector<glow::vec4> data(static_cast<uint32_t>(
      image.GetCamera().GetWidth() * image.GetCamera().GetHeight()));
  depth_projector_->texture().download(data);
  std::shared_ptr<ProcessedImage> processed_img(
      new ProcessedImage(image, data));
  image_sequence_->AddImage(processed_img, kernel_size);
}

std::vector<PointCovariance3d> ChangeDetector::GetChanges() {
  Regions3dProjector projector(image_sequence_, options_.threshold_change_area,
                               options_.chi_square2d, options_.chi_square3d);
  // Get the bounding box from the mesh and the cameras and discard the regions
  // outside it
  std::pair<Eigen::Vector3d, Eigen::Vector3d> bounding_box =
      mesh_.GetBoundingBox();
  for (int i = 0; i < image_sequence_->size(); i++) {
    if ((*image_sequence_)[i].GetCamera().GetPosition()(0) <
        bounding_box.first(0))
      bounding_box.first(0) =
          (*image_sequence_)[i].GetCamera().GetPosition()(0);
    if ((*image_sequence_)[i].GetCamera().GetPosition()(1) <
        bounding_box.first(1))
      bounding_box.first(1) =
          (*image_sequence_)[i].GetCamera().GetPosition()(1);
    if ((*image_sequence_)[i].GetCamera().GetPosition()(2) <
        bounding_box.first(2))
      bounding_box.first(2) =
          (*image_sequence_)[i].GetCamera().GetPosition()(2);
    if ((*image_sequence_)[i].GetCamera().GetPosition()(0) >
        bounding_box.second(0))
      bounding_box.second(0) =
          (*image_sequence_)[i].GetCamera().GetPosition()(0);
    if ((*image_sequence_)[i].GetCamera().GetPosition()(1) >
        bounding_box.second(1))
      bounding_box.second(1) =
          (*image_sequence_)[i].GetCamera().GetPosition()(1);
    if ((*image_sequence_)[i].GetCamera().GetPosition()(2) >
        bounding_box.second(2))
      bounding_box.second(2) =
          (*image_sequence_)[i].GetCamera().GetPosition()(2);
  }
  std::vector<PointCovariance3d> regions3d = projector.GetProjectedRegions();
  std::vector<PointCovariance3d> regions3d_in_model;
  for (auto &region : regions3d) {
    if (region.Point()(0) > bounding_box.first(0) &&
        region.Point()(1) > bounding_box.first(1) &&
        region.Point()(2) > bounding_box.first(2) &&
        region.Point()(0) < bounding_box.second(0) &&
        region.Point()(1) < bounding_box.second(1) &&
        region.Point()(2) < bounding_box.second(2)) {
      std::vector<Eigen::Vector3d> vertices = region.Vertices();
      bool out = false;
      for (auto &v : vertices){
        if(v(0) < bounding_box.first(0) ||
        v(1) < bounding_box.first(1) ||
        v(2) < bounding_box.first(2) ||
        v(0) > bounding_box.second(0) ||
        v(1) > bounding_box.second(1) ||
        v(2) > bounding_box.second(2)){
          break;
        }
      }
      if (!out) {
        regions3d_in_model.push_back(region);
      }
    }
  }
  return regions3d_in_model;
}

}  // namespace fastcd
