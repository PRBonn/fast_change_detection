// Copyright 2017 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de), Cyrill Stachniss, University of Bonn
#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <glow/glutil.h>
#include <opencv2/core/core.hpp>
#include "fastcd/camera.h"
#include "fastcd/point_covariance2d.h"
#include "fastcd/image.h"
#include "fastcd/image_region.h"

namespace fastcd {

/**
 * @brief      Class that stores an image, its camera calibration, and all the 
 *             necessary information for the change detection algorithm. It also
 *             includes the algorithms to generate such information.
 */ 
class ProcessedImage : public Image {
 public:
  /**
   * @brief      Constructor.
   *
   * @param[in]  img    The image
   * @param[in]  depth  The depth image w.r.t. the 3D model
   */
  explicit ProcessedImage(const Image &img,
                          const std::vector<glow::vec4> &depth);
  /**
   * @brief      Warps the image using camera calibration of the provided image
   *             and the depth. 
   *
   * @param[in]  target_image  The target image
   *
   * @return     The warped image
   */
  Image Warp(const ProcessedImage &target_image) const;

  /**
   * @brief      Update the inconsistency image by comparing this image with the
   *             one passed as input (it computes the minimum difference over a
   *             window of size kernel_size).
   *
   * @param[in]  image  The new image to be compared
   * @param[in]  kernel_size The size of the window over which the minimum
   *                         difference is computed
   */
  void UpdateInconsistencies(const Image &image, int kernel_size);

  /**
   * @brief      Gets the number of images compared with this one.
   *
   * @return     The number of images compared with this one.
   */
  int ImageCompared();

  /**
   * @brief      Calculates the mean position and covariance of the points of
   *             each region with the same label.
   *
   * @param[in]  chi_square  The chi square value (95% of the variance retained
   *                          by default)
   *
   * @return     The labels of the processed regions.
   */
  std::vector<int> ComputeMeansCovariances(double chi_square = 5.991);

  /**
   * @brief      Extract the regions where changes occur from the inconsistency
   *             image.
   * 
   * @param[in]  threshold_change_area Threshold area under which regions are
   * not considered
   */ 
  void ComputeRegions(int threshold_change_area);

  /**
   * @brief      Gets the mean position and covariance of the points of each
   *             region with the same label.
   *
   * @return     The map containing the pairs [label, XXX].
   */
  std::unordered_map<int, PointCovariance2d> GetMeansCovariances() const;

  /**
   * @brief      Gets the inconsistency image.
   *
   * @return     The inconsistency image.
   */
  cv::Mat GetInconsistencies();

  /**
   * @brief      Gets the regions where changes occur.
   *
   * @return     The regions.
   */
  const std::vector<ImageRegion>& GetRegions() const;

  /**
   * @brief      Gets the depth image.
   *
   * @return     The depth image.
   */
  const std::vector<glow::vec4>& GetDepth() const;

  /**
   * @brief      Update the label of the regions with the provided values.
   *
   * @param[in]  labels  The new labels
   */
  void UpdateLabels(const std::vector<int> &labels);

 protected:
  /**
   * @brief      For every pixel of img1 subtracts the color of img2 in a window
   *             of kernel_size pixel and stores the smallest L2 norm of the
   *             result in a OpenCV matrix of doubles.
   *
   * @param[in]  img1  The first image
   * @param[in]  img2  The second image
   * @param[in]  kernel_size The size in pixel of the window in img2 over which
   *                         the difference is computed
   *
   * @return     The resulting OpenCV matrix.
   */
  cv::Mat Subtract(const cv::Mat &img1, const cv::Mat &img2, int kernel_size);

  /**
   * @brief      Computes the minimum element-wise of two OpenCV matrices of
   * doubles.
   *
   * @param[in]  img1  The first matrix
   * @param[in]  img2  The second matrix
   *
   * @return     The resulting matrix
   */
  cv::Mat Minimum(const cv::Mat &img1, const cv::Mat &img2);

  /**
   * @brief      Find non-zero elements in the matrix
   *
   * @param[in]  img   The matrix
   *
   * @return     The (rows*cols) by 2 matrix containing all the non-zero
   *             elements.
   */
  cv::Mat FindNonZero(const cv::Mat &img);

  /**
   * @brief      Segments the inconsistency image into region and fills those
   *             regions with their average intensity.
   *
   * @param[in]  img   The inconsistency image
   * @param[in]  threshold_change_area Threshold area under which regions are
   * not considered
   *
   * @return     The inconsistency image with the segmented regions.
   */
  cv::Mat AverageSegmentedRegions(const cv::Mat &img, int threshold_change_area);

  /** 
   * The depth image w.r.t. the 3D model, in the form of a vector of 4D points
   * ordered row-wise from the bottom left. The 4th coordinate is used as a
   * boolean: it has value 0 if the back-projected ray does not hit the model,
   * 1 otherwise
   */
  std::vector<glow::vec4> depth_;

  /** The inconsistency image (doubles from 0 to 1) */
  cv::Mat inconsistencies_;

  /** The segmented regions */
  std::vector<ImageRegion> regions_;

  /** 
   * The mean position and covariance of the points of each region with the same
   * label
   */
  std::unordered_map<int, PointCovariance2d> regions_mean_cov_;

  /** The highest label assigned to a region */
  int max_region_label_ = 0;

  /** The number of images this one has been compared to*/
  int num_img_compared_ = 0;
};

}  // namespace fastcd
