// Copyright 2017 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de)
#pragma once

#include <vector>
#include "fastcd/processed_image.h"
#include "fastcd/regions_matcher.h"

namespace fastcd {

/**
 * @brief      Class for matching regions using histograms of their hue and
 *             saturation values.
 */
class RegionsMatcherHist : RegionsMatcher {
 public:
  /**
   * @brief      Matches the regions of the two images and computes the relarive
   *             labels.
   *
   * @param[in]  img1       The first image
   * @param[in]  img2       The second image
   * @param[in]  min_label  The minimum label to assign
   */
  void Match(const ProcessedImage &img1, const ProcessedImage &img2,
             int min_label) override;

  /**
   * @brief      Gets the computed labels for the regions of the first image.
   *
   * @return     The vector of labels.
   */
  std::vector<int> GetLabelsImg1() override;

  /**
   * @brief      Gets the computed labels for the regions of the second image.
   *
   * @return     The vector of labels.
   */
  std::vector<int> GetLabelsImg2() override;

  /**
   * @brief      Gets the maximum assigned label.
   *
   * @return     The maximum assigned label.
   */
  int GetMaxLabel() override;

 protected:
  /**
   * @brief      Computes a mask that represents the epipolar constraints of the
   *             region idx of img1 on img2.
   *
   * @param[in]  img1  The first image
   * @param[in]  img2  The second image
   * @param[in]  idx   The index of the region of the first image for which the
   *                   mask is created
   *
   * @return     The mask.
   */
  cv::Mat EpilineMask(const ProcessedImage &img1, const ProcessedImage &img2,
                      int idx);

  /**
   * @brief      Calculates the fundamental matrix given the projection matrices
   *             of two cameras.
   *
   * @param[in]  P1    The projection matrix of the first camera
   * @param[in]  P2    The projection matrix of the second camera
   *
   * @return     The fundamental matrix.
   */
  Eigen::Matrix3d ComputeF(const Eigen::Matrix<double, 3, 4> &P1,
                           const Eigen::Matrix<double, 3, 4> &P2);

  /** The labels of the regions of the first image */
  std::vector<int> labels1_;

  /** The labels of the regions of the second image */
  std::vector<int> labels2_;

  /** The maximum assigned label */
  int max_label_;
};

}  // namespace fastcd
