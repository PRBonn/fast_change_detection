// Copyright 2017 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de)
#include "fastcd/processed_image.h"
#include <algorithm>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>
#include "fastcd/camera.h"

namespace fastcd {

ProcessedImage::ProcessedImage(const Image &img,
                               const std::vector<glow::vec4> &depth)
    : Image(img), depth_(depth) {
  inconsistencies_ =
        cv::Mat(raw_image_.rows, raw_image_.cols, CV_32FC1, cv::Scalar(0));
}

Image ProcessedImage::Warp(const ProcessedImage &target_image) const {
  cv::Mat result(raw_image_.rows, raw_image_.cols, raw_image_.type(),
                 cv::Scalar(0, 0, 0));
  cv::Mat zbuffer(raw_image_.rows / 2, raw_image_.cols / 2, CV_64FC1,
                  cv::Scalar(1000));
  Eigen::Vector2i coord;
  #pragma omp parallel for collapse(2)
  for (int i = 0; i < raw_image_.rows; ++i) {
    for (int j = 0; j < raw_image_.cols; j++) {
      int idx = i * raw_image_.cols + j;
      if (depth_[idx].w > 0.5) {
        coord = target_image.GetCamera().Project(
            static_cast<double>(depth_[idx].x),
            static_cast<double>(depth_[idx].y),
            static_cast<double>(depth_[idx].z));
        // Check if the point is seen in both images
        // (taking into account occlusions)
        if (coord(1) > 0 && coord(1) < raw_image_.rows && coord(0) > 0 &&
            coord(0) < raw_image_.cols) {
          double distance =
              sqrt(pow(static_cast<double>(depth_[idx].x) -
                           target_image.GetCamera().GetPosition()(0), 2) +
                   pow(static_cast<double>(depth_[idx].y) -
                           target_image.GetCamera().GetPosition()(1), 2) +
                   pow(static_cast<double>(depth_[idx].z) -
                           target_image.GetCamera().GetPosition()(2), 2));
          if (distance < zbuffer.at<double>(coord(1) / 2, coord(0) / 2)) {
            zbuffer.at<double>(coord(1) / 2, coord(0) / 2) = distance;
            int depth_idx =
                (raw_image_.rows - coord(1) - 1) * target_image.Width() +
                coord(0);
            double dist_target =
                sqrt(pow(target_image.GetCamera().GetPosition()(0) -
                             target_image.GetDepth()[depth_idx].x, 2) +
                     pow(target_image.GetCamera().GetPosition()(1) -
                             target_image.GetDepth()[depth_idx].y, 2) +
                     pow(target_image.GetCamera().GetPosition()(2) -
                             target_image.GetDepth()[depth_idx].z, 2));
            if (dist_target >= distance - 0.5) {
              result.at<cv::Vec3b>(coord(1), coord(0)) =
                  raw_image_.at<cv::Vec3b>(raw_image_.rows - i - 1, j);
            }
          }
        }
      }
    }
  }

  // Interpolation
  cv::Mat interpolation(result.rows, result.cols, result.type());
  int dilation_size = 1;
  cv::Mat dilation_kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
      cv::Point(dilation_size, dilation_size));
  cv::dilate(result, interpolation, dilation_kernel);
  for (int i = 0; i < result.rows; i++) {
    for (int j = 0; j < result.cols; j++) {
      if (result.at<cv::Vec3b>(i, j) == cv::Vec3b(0, 0, 0)) {
        result.at<cv::Vec3b>(i, j) = interpolation.at<cv::Vec3b>(i, j);
      }
    }
  }
  return Image(result, target_image.GetCamera());
}

void ProcessedImage::UpdateInconsistencies(const Image &image,
                                           int kernel_size) {
  cv::Mat inconsistency =
      Subtract(raw_image_, image.GetRawImage(), kernel_size);

  cv::normalize(inconsistency, inconsistency, 0.0f, 1.0f, cv::NORM_MINMAX);
  if (num_img_compared_ > 0)
    inconsistencies_ = Minimum(inconsistencies_, inconsistency);
  else
    inconsistencies_ = inconsistency;
  cv::medianBlur(inconsistencies_, inconsistencies_, 3);
  cv::normalize(inconsistencies_, inconsistencies_, 0.0f, 1.0f,
                cv::NORM_MINMAX);

  num_img_compared_++;
}

cv::Mat ProcessedImage::GetInconsistencies() { return inconsistencies_; }

std::vector<int> ProcessedImage::ComputeMeansCovariances(double chi_square) {
  std::vector<int> found_labels;
  for (int i = 0; i <= max_region_label_; i++) {
    std::vector<int> idx;
    for (size_t j = 0; j < regions_.size(); j++) {
      if (regions_[j].label_ == i) {
        idx.push_back(j);
      }
    }
    if (!idx.empty()) {
      found_labels.push_back(i);
      cv::Mat region(Height(), Width(), CV_8UC1, cv::Scalar(0));
      std::vector<std::vector<cv::Point>> contours;
      for (int j : idx) {
        contours.push_back(regions_[j].contour_);
      }
      cv::drawContours(region, contours, -1, cv::Scalar(255), CV_FILLED);
      cv::Mat non_zero = FindNonZero(region);
      cv::Mat mean;
      cv::Mat covariance(2, 2, CV_64F);
      cv::calcCovarMatrix(non_zero, covariance, mean,
                        CV_COVAR_NORMAL | CV_COVAR_ROWS | CV_COVAR_SCALE);
      Eigen::Vector2d emean;
      Eigen::Matrix2d ecovariance;
      emean << mean.at<double>(0, 0), mean.at<double>(0, 1);
      ecovariance << covariance.at<double>(0, 0), covariance.at<double>(0, 1),
      covariance.at<double>(1, 0), covariance.at<double>(1, 1);
      PointCovariance2d mean_covariance(emean, ecovariance, chi_square);
      regions_mean_cov_[i] = mean_covariance;
    }
  }
  return found_labels;
}

void ProcessedImage::ComputeRegions(int threshold_change_area) {
  cv::Mat image(Height(), Width(), CV_8UC1);
  inconsistencies_.convertTo(image, CV_8UC1, 255);
  cv::threshold(image, image, 30, 255.0f, cv::THRESH_TOZERO);

  cv::Mat dilation_kernel =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11));
  cv::Mat erosion_kernel =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  cv::erode(image, image, erosion_kernel);
  cv::dilate(image, image, dilation_kernel);
  image = AverageSegmentedRegions(image, threshold_change_area);

  cv::equalizeHist(image, image);
  cv::normalize(image, image, 0, 255, cv::NORM_MINMAX);
  cv::threshold(image, image, 50, 255.0f, cv::THRESH_TOZERO);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  for (size_t i = 0; i < contours.size(); i++) {
    ImageRegion region(contours[i]);
    regions_.push_back(region);
  }
}

const std::vector<ImageRegion>& ProcessedImage::GetRegions() const {
  return regions_;
}

const std::vector<glow::vec4>& ProcessedImage::GetDepth() const {
  return depth_;
}

void ProcessedImage::UpdateLabels(const std::vector<int> &labels) {
  for (size_t i = 0; i < regions_.size(); i++) {
    if (labels[i] > max_region_label_) max_region_label_ = labels[i];
    regions_[i].label_ = labels[i];
  }
}

cv::Mat ProcessedImage::AverageSegmentedRegions(const cv::Mat &img,
                                                int threshold_change_area) {
  cv::Mat labels(img.rows, img.cols, img.type(), cv::Scalar(0));
  cv::Mat dst(img.rows, img.cols, img.type(), cv::Scalar(0));
  cv::Mat tmp = img.clone();
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(tmp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  std::vector<float> cont_avgs(contours.size(), 0.f);
  for (size_t i = 0; i < contours.size(); ++i) {
    cv::drawContours(labels, contours, i, cv::Scalar(i + 1), CV_FILLED);
    cv::Rect roi = cv::boundingRect(contours[i]);
    cv::Mat mask;
    cv::bitwise_and(labels(roi) == (i + 1), img(roi) != 0, mask);
    cv::Scalar mean = cv::mean(img(roi), mask);
    cont_avgs[i] = mean[0];
  }
  for (size_t i = 0; i < contours.size(); ++i) {
    if (cv::contourArea(contours[i]) > threshold_change_area)
      cv::drawContours(dst, contours, i, cv::Scalar(cont_avgs[i]), CV_FILLED);
  }
  return dst;
}

std::unordered_map<int, PointCovariance2d>
ProcessedImage::GetMeansCovariances() const {
  return regions_mean_cov_;
}

cv::Mat ProcessedImage::Subtract(const cv::Mat &img1, const cv::Mat &img2,
                                 int kernel_size) {
  cv::Mat result(img1.rows, img1.cols, CV_32FC1, 0.0f);

  for (int i = 0; i < img1.rows; ++i) {
    for (int j = 0; j < img1.cols; ++j) {
      Eigen::Vector3f img1_color, img2_color;
      img1_color << static_cast<float>(img1.at<cv::Vec3b>(i, j)[0]),
          static_cast<float>(img1.at<cv::Vec3b>(i, j)[1]),
          static_cast<float>(img1.at<cv::Vec3b>(i, j)[2]);
      if (img1_color.norm() > 0.01) {
        if (j - kernel_size / 2 > 0 &&
            i - kernel_size / 2 > 0 &&
            j + kernel_size / 2 < img2.cols &&
            i + kernel_size / 2 < img2.rows) {
          cv::Rect window(j - kernel_size / 2,
                          i - kernel_size / 2, kernel_size,
                          kernel_size);
          // Loop on the window
          float min = 10000;
          for (int subi = 0; subi < window.height; subi++) {
            for (int subj = 0; subj < window.width; subj++) {
              cv::Mat area = img2(window);
              img2_color << static_cast<float>(
                  area.at<cv::Vec3b>(subi, subj)[0]),
                  static_cast<float>(area.at<cv::Vec3b>(subi, subj)[1]),
                  static_cast<float>(area.at<cv::Vec3b>(subi, subj)[2]);
            // Compare the values here and keep track of the minimum difference
              if (img2_color.norm() > 0.01) {
                float difference = (img1_color - img2_color).norm();
                if (difference < min) min = difference;
              }
            }
          }
          if (min < 1000) result.at<float>(i, j) = min;
        }
      }
    }
  }
  return result;
}

cv::Mat ProcessedImage::Minimum(const cv::Mat &img1, const cv::Mat &img2) {
  cv::Mat result(img1.rows, img1.cols, CV_32FC1, 0.0f);
  #pragma omp parallel for collapse(2)
  for (int i = 0; i < img1.rows; ++i) {
    for (int j = 0; j < img1.cols; ++j) {
      result.at<float>(i, j) =
          std::min(img1.at<float>(i, j), img2.at<float>(i, j));
    }
  }
  return result;
}

cv::Mat ProcessedImage::FindNonZero(const cv::Mat &img) {
  cv::Mat result(img.rows * img.cols, 2, CV_64F);
  int idx = 0;
  for (int i = 0; i < img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {
      if (img.at<uchar>(i, j) > 0) {
        result.at<double>(idx, 0) = j;
        result.at<double>(idx, 1) = i;
        idx++;
      }
    }
  }
  return result(cv::Rect(0, 0, 2, idx));
}

int ProcessedImage::ImageCompared() { return num_img_compared_; }

}  // namespace fastcd
