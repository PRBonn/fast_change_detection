// Copyright 2017 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de)
#pragma once

#include <memory>
#include <vector>
#include "fastcd/point_covariance3d.h"
#include "fastcd/image.h"
#include "fastcd/image_sequence.h"
#include "fastcd/depth_projector.h"
#include "fastcd/mesh.h"

namespace fastcd {

/**
 * @brief      Main class of the change detector algorithm.
 */
class ChangeDetector {
 public:
  /**
   * @brief      Options for the change detector algorithm (see ChangeDetector).
   */
  struct ChangeDetectorOptions {
    /** The maximum amount of images stored in the queue. */
    int cache_size;

    /** The maximum amount of comparisons per image. */
    int max_comparisons;

    /** Chi square value for 2D confidence ellipses (95% of the variance
     * retained by default).
     */
    double chi_square2d = 5.991;

    /** Chi square value for 3D confidence ellipsoids (95% of the variance
     * retained by default).
     */
    double chi_square3d = 7.815;

    /** Width to which rescale the images */
    int rescale_width;

    /** The threshold area under which a 2D change is discarded */
    int threshold_change_area;
  };

  /**
   * @brief      Constructor.
   *
   * @param[in]  mesh     The 3D model of the environment
   * @param[in]  options  The options of the algorithm (see ChangeDetectorOptions)
   */
  explicit ChangeDetector(const Mesh &mesh,
                          const ChangeDetectorOptions &options);

  /**
   * @brief      Adds an image to the sequence and process it.
   *
   * @param[in]  image  The image
   * @param[in]  kernel_size The size of the window over which the image is
   * compared with the others. It represents the uncertainty of the camera pose
   */
  void AddImage(Image &image, int kernel_size);

  /**
   * @brief      Gets the detected changes.
   *
   * @return     The changes.
   */
  std::vector<PointCovariance3d> GetChanges();

 protected:
  /** The 3D model of the environment */
  Mesh mesh_;

  /** The struct containing the options of the algorithm */
  ChangeDetectorOptions options_;

  /** It stores whether the depth projector has been initialized or not */
  bool projector_init_ = false;

  /** The sequence of images */
  std::shared_ptr<ImageSequence> image_sequence_;

  /** The depth projector */
  std::unique_ptr<DepthProjector> depth_projector_;
};

}  // namespace fastcd
