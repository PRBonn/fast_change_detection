// Copyright 2017 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de), Cyrill Stachniss, University of Bonn
#pragma once
#include <vector>
#include <opencv2/core/core.hpp>

namespace fastcd {

/**
 * @brief      Class that represents a labeled region.
 */
class ImageRegion {
 public:
  /**
   * @brief      Constructor.
   *
   * @param[in]  contour  The contour of the region
   */
  explicit ImageRegion(std::vector<cv::Point> contour)
      : contour_(contour) {}

  /** The points describing the contour of the region */
  std::vector<cv::Point> contour_;

  /** The label of the region */
  int label_ = -1;
};

}  // namespace fastcd
