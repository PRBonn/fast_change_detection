// Copyright 2017 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de)
#pragma once

#include <string>
#include <opencv2/core/core.hpp>
#include "fastcd/camera.h"

namespace fastcd {

/**
 * @brief      Class that stores an image and its calibration.
 */
class Image {
 public:
  /**
   * @brief      Empty constructor.
   */
  Image();

  /**
   * @brief      Constructor from OpenCV image.
   *
   * @param[in]  img       The OpenCV image
   * @param[in]  camera    The camera calibration
   */
  explicit Image(const cv::Mat &img, const Camera &camera);

  /**
   * @brief      Copy constructor.
   *
   * @param[in]  img   The image
   */
  Image(const Image &img);

  /**
   * @brief      Loads an image from a file, with a passed camera calibration.
   *
   * @param[in]  filepath  The image file path
   * @param[in]  camera    The camera calibration
   *
   * @return     true if the load was successful, false otherwise.
   */
  bool LoadImage(std::string filepath, const Camera &camera);

  /**
   * @brief      Scales the image and its calibration by the specified factor.
   *
   * @param[in]  factor  The scaling factor
   */
  void Scale(double factor);

  /**
   * @brief      Gets the camera calibration.
   *
   * @return     The camera calibration.
   */
  Camera GetCamera() const;

  /**
   * @brief      Gets the raw OpenCV image.
   *
   * @return     The raw image.
   */
  cv::Mat GetRawImage() const;

  /**
   * @brief      Gets the width of the image in pixel.
   *
   * @return     The width of the image.
   */
  int Width() const;

  /**
   * @brief      Gets the height of the image in pixel.
   *
   * @return     The height of the image.
   */
  int Height() const;

 protected:
  /** The camera calibration */
  Camera camera_;

  /** The OpenCV image*/
  cv::Mat raw_image_;
};

}  // namespace fastcd
