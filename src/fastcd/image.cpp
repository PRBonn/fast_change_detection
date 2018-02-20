// Copyright 2017 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de)
#include "fastcd/image.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace fastcd {

Image::Image() {}

Image::Image(const cv::Mat &img, const Camera &camera)
    : camera_(camera), raw_image_(img) {}

Image::Image(const Image &img)
    : camera_(img.camera_), raw_image_(img.raw_image_) {}

Camera Image::GetCamera() const { return camera_; }

cv::Mat Image::GetRawImage() const { return raw_image_; }

bool Image::LoadImage(std::string filepath, const Camera &camera) {
  raw_image_ = cv::imread(filepath, cv::IMREAD_COLOR);
  if (!raw_image_.data) {
    return false;
  } else {
    camera_ = camera;
    return true;
  }
}

void Image::Scale(double factor) {
  camera_.ScaleCalibration(factor);
  cv::resize(raw_image_, raw_image_,
             cv::Size(Width() * factor, Height() * factor));   
}

int Image::Width() const { return raw_image_.cols; }
int Image::Height() const { return raw_image_.rows; }

}  // namespace fastcd
