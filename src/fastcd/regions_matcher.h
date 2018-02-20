// Copyright 2017 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de)
#pragma once

#include <vector>
#include "fastcd/processed_image.h"

namespace fastcd {

/**
 * @brief      Pure abstract class for a regions matcher. A region matcher
 *             performs the data association between the segmented regions of
 *             two images and returns the labels assigned to the different regions.
 */
class RegionsMatcher {
 public:
  /**
   * @brief      Matches the regions of the two images and computes the relarive
   *             labels.
   *
   * @param[in]  img1       The first image
   * @param[in]  img2       The second image
   * @param[in]  min_label  The minimum label to assign
   */
  virtual void Match(const ProcessedImage &img1, const ProcessedImage &img2,
                     int min_label) = 0;
  /**
   * @brief      Gets the computed labels for the regions of the first image.
   *
   * @return     The vector of labels.
   */
  virtual std::vector<int> GetLabelsImg1() = 0;

  /**
   * @brief      Gets the computed labels for the regions of the second image.
   *
   * @return     The vector of labels.
   */
  virtual std::vector<int> GetLabelsImg2() = 0;

  /**
   * @brief      Gets the maximum assigned label.
   *
   * @return     The maximum assigned label.
   */
  virtual int GetMaxLabel() = 0;
};

}  // namespace fastcd
