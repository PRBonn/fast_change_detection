// Copyright 2017 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de), Cyrill Stachniss, University of Bonn
#include "fastcd/regions_matcher_hist.h"
#include <algorithm>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>

namespace fastcd {

void RegionsMatcherHist::Match(const ProcessedImage &img1,
                           const ProcessedImage &img2, int min_label) {
  max_label_ = min_label;
  std::vector<ImageRegion> regions_img1 = img1.GetRegions();
  std::vector<ImageRegion> regions_img2 = img2.GetRegions();
  labels1_.resize(regions_img1.size());
  labels2_.resize(regions_img2.size());
  std::vector<std::vector<cv::Point>> contours(1);
  std::unordered_map<int, cv::Mat> epimasks1;
  std::vector<cv::MatND> img1_hists(regions_img1.size());
  std::vector<cv::MatND> img2_hists(regions_img2.size());
  for (size_t i = 0; i < regions_img1.size(); i++) {
    labels1_[i] = regions_img1[i].label_;
    cv::Mat epimask = EpilineMask(img1, img2, i);
    epimasks1[i] = epimask;
    contours[0] = regions_img1[i].contour_;
    cv::Mat mask(img1.Height(), img1.Width(), CV_8UC1, cv::Scalar(0));
    cv::drawContours(mask, contours, -1, cv::Scalar(255), CV_FILLED);
    cv::Mat hsv;
    cv::cvtColor(img1.GetRawImage(), hsv, cv::COLOR_BGR2HSV);
    int h_bins = 50; int s_bins = 60;
    int histSize[] = { h_bins, s_bins };
    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };
    const float* ranges[] = { h_ranges, s_ranges };
    int channels[] = { 0, 1 };
    calcHist(&hsv, 1, channels, mask, img1_hists[i], 2, histSize, ranges, true,
             false);
    normalize(img1_hists[i], img1_hists[i], 0, 1, cv::NORM_MINMAX, -1,
              cv::Mat());
  }
  std::unordered_map<int, cv::Mat> epimasks2;
  for (size_t i = 0; i < regions_img2.size(); i++) {
    labels2_[i] = regions_img2[i].label_;
    cv::Mat epimask = EpilineMask(img2, img1, i);
    epimasks2[i] = epimask;
    contours[0] = regions_img2[i].contour_;
    cv::Mat mask(img2.Height(), img2.Width(), CV_8UC1, cv::Scalar(0));
    cv::drawContours(mask, contours, -1, cv::Scalar(255), CV_FILLED);
    cv::Mat hsv;
    cv::cvtColor(img2.GetRawImage(), hsv, cv::COLOR_BGR2HSV);
    int h_bins = 50; int s_bins = 60;
    int histSize[] = { h_bins, s_bins };
    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };
    const float* ranges[] = { h_ranges, s_ranges };
    int channels[] = { 0, 1 };
    calcHist(&hsv, 1, channels, mask, img2_hists[i], 2, histSize,
             ranges, true, false);
    normalize(img2_hists[i], img2_hists[i], 0, 1,
              cv::NORM_MINMAX, -1, cv::Mat());
  }
  std::vector<int> idxs;
  std::vector<double> corrs;
  for (size_t i = 0; i < regions_img1.size(); i++) {
    double max = 0;
    int maxj;
    bool geomeric_check1, geomeric_check2;
    for (size_t j = 0; j < regions_img2.size(); j++) {
      geomeric_check1 = false;
      geomeric_check2 = false;
      std::vector<cv::Point> contour = regions_img2[j].contour_;
      for (auto &pt : contour) {
        if (epimasks1[i].at<uchar>(pt.y, pt.x) == 0) {
          geomeric_check1 = true;
          break;
        }
      }
      contour = regions_img1[i].contour_;
      for (auto &pt : contour) {
        if (epimasks2[j].at<uchar>(pt.y, pt.x) == 0) {
          geomeric_check2 = true;
          break;
        }
      }

      double corr;
      if (geomeric_check1 || geomeric_check2) {
        corr = 0;
      } else {
        corr = cv::compareHist(img1_hists[i],
                                      img2_hists[j], CV_COMP_CORREL);
      }
      if (corr > max) {
        max = corr;
        maxj = j;
      }
    }
    idxs.push_back(maxj);
    corrs.push_back(max);
  }

  double threshold, max = 0, min = 1;
  for (size_t i = 0; i < corrs.size(); i++) {
    if (corrs[i] > max) max = corrs[i];
    if (corrs[i] < min) min = corrs[i];
  }
  threshold = std::max((max - min) * 0.7 + min, 0.5);
  for (size_t i = 0; i < regions_img1.size(); i++) {
    if (corrs[i] >= threshold) {
      if (labels2_[idxs[i]] < 0) {
        labels2_[idxs[i]] = labels1_[i] = max_label_++;
      } else {
        labels1_[i] = labels2_[idxs[i]];
      }
    }
  }
}

std::vector<int> RegionsMatcherHist::GetLabelsImg1() {
  return labels1_;
}

std::vector<int> RegionsMatcherHist::GetLabelsImg2() {
  return labels2_;
}

int RegionsMatcherHist::GetMaxLabel() { return max_label_; }

cv::Mat RegionsMatcherHist::EpilineMask(const ProcessedImage &img1,
                                    const ProcessedImage &img2, int idx) {
  Eigen::Matrix3d F = ComputeF(img1.GetCamera().GetP(),
                               img2.GetCamera().GetP());
  cv::Mat epimask(img1.Height(), img1.Width(), CV_8UC1,
                  cv::Scalar(0));
  std::vector<ImageRegion> regions_img1 = img1.GetRegions();
  for (size_t i = 0; i < regions_img1[idx].contour_.size(); i++) {
    Eigen::Vector3d pt;
    pt << regions_img1[idx].contour_[i].x,
        regions_img1[idx].contour_[i].y, 1;
    Eigen::Vector3d line = F * pt;
    cv::line(
        epimask, cv::Point(0, -line(2) / line(1)),
        cv::Point(epimask.cols, -(line(2) + line(0) * epimask.cols) / line(1)),
        cv::Scalar(255));
  }
  cv::Mat dilation_kernel =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(51, 51));
  cv::dilate(epimask, epimask, dilation_kernel);
  return epimask;
}

Eigen::Matrix3d RegionsMatcherHist::ComputeF(
    const Eigen::Matrix<double, 3, 4> &P1,
    const Eigen::Matrix<double, 3, 4> &P2) {
  Eigen::Matrix<double, 2, 4> X1, X2, X3, Y1, Y2, Y3;

  X1.block<1, 4>(0, 0) = P1.block<1, 4>(1, 0);
  X1.block<1, 4>(1, 0) = P1.block<1, 4>(2, 0);

  X2.block<1, 4>(0, 0) = P1.block<1, 4>(2, 0);
  X2.block<1, 4>(1, 0) = P1.block<1, 4>(0, 0);

  X3.block<1, 4>(0, 0) = P1.block<1, 4>(0, 0);
  X3.block<1, 4>(1, 0) = P1.block<1, 4>(1, 0);

  Y1.block<1, 4>(0, 0) = P2.block<1, 4>(1, 0);
  Y1.block<1, 4>(1, 0) = P2.block<1, 4>(2, 0);

  Y2.block<1, 4>(0, 0) = P2.block<1, 4>(2, 0);
  Y2.block<1, 4>(1, 0) = P2.block<1, 4>(0, 0);

  Y3.block<1, 4>(0, 0) = P2.block<1, 4>(0, 0);
  Y3.block<1, 4>(1, 0) = P2.block<1, 4>(1, 0);

  Eigen::Matrix4d XY11, XY21, XY31, XY12, XY22, XY32, XY13, XY23, XY33;

  XY11.block<2, 4>(0, 0) = X1;
  XY11.block<2, 4>(2, 0) = Y1;

  XY21.block<2, 4>(0, 0) = X2;
  XY21.block<2, 4>(2, 0) = Y1;

  XY31.block<2, 4>(0, 0) = X3;
  XY31.block<2, 4>(2, 0) = Y1;

  XY12.block<2, 4>(0, 0) = X1;
  XY12.block<2, 4>(2, 0) = Y2;

  XY22.block<2, 4>(0, 0) = X2;
  XY22.block<2, 4>(2, 0) = Y2;

  XY32.block<2, 4>(0, 0) = X3;
  XY32.block<2, 4>(2, 0) = Y2;

  XY13.block<2, 4>(0, 0) = X1;
  XY13.block<2, 4>(2, 0) = Y3;

  XY23.block<2, 4>(0, 0) = X2;
  XY23.block<2, 4>(2, 0) = Y3;

  XY33.block<2, 4>(0, 0) = X3;
  XY33.block<2, 4>(2, 0) = Y3;

  Eigen::Matrix3d F;
  F << XY11.determinant(), XY21.determinant(), XY31.determinant(),
       XY12.determinant(), XY22.determinant(), XY32.determinant(),
       XY13.determinant(), XY23.determinant(), XY33.determinant();
  return F;
}



}  // namespace fastcd
