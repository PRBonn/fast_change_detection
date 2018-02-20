// Copyright 2017 Jens Behley (jens.behley@igg.uni-bonn.de)
#pragma once

#include <stdint.h>
#include <eigen3/Eigen/Dense>

#include <glow/GlFramebuffer.h>
#include <glow/GlProgram.h>
#include <glow/GlTexture.h>
#include "fastcd/mesh.h"

namespace fastcd {

/**
 * @brief      Basic projection into a framebuffer texture.
 */
class DepthProjector {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief      Constructor.
   *
   * @param[in]  width       The image width
   * @param[in]  height      The image height
   * @param[in]  projection  The camera projection matrix
   */
  DepthProjector(uint32_t width, uint32_t height,
                 const Eigen::Matrix4f& projection);

  /**
   * @brief      Renders the given mesh from the given view relative to the
   *             mesh.
   *
   * @param[in]  mesh  The mesh
   * @param[in]  view  The view
   */
  void render(const Mesh& mesh, const Eigen::Matrix4f& view);

  /**
   * @brief      Gets the depth projection.
   *
   * @return     The depth image.
   */
  glow::GlTexture& texture();

 protected:
  /** The framebuffer used to render the mesh */
  glow::GlFramebuffer framebuffer_;

  /** The output texture */
  glow::GlTexture output_;

  /** The program used to render the mesh */
  glow::GlProgram program_;

  /** The projection matrix of the cameras*/
  Eigen::Matrix4f projection_;
};

}  // namespace fastcd
