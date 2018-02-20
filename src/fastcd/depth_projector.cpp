// Copyright 2017 Jens Behley (jens.behley@igg.uni-bonn.de)
#include "fastcd/depth_projector.h"
#include "fastcd/mesh.h"

namespace fastcd {

DepthProjector::DepthProjector(uint32_t width, uint32_t height,
                               const Eigen::Matrix4f& projection)
    : framebuffer_(width, height),
      output_(width, height, glow::TextureFormat::RGBA_FLOAT),
      projection_(projection) {
  framebuffer_.attach(glow::FramebufferAttachment::COLOR0, output_);
  glow::GlRenderbuffer rbo(width, height,
                           glow::RenderbufferFormat::DEPTH_STENCIL);
  framebuffer_.attach(glow::FramebufferAttachment::DEPTH_STENCIL,
                      rbo);  // framebuffers needs a depth/stencil buffer.

  program_.attach(glow::GlShader::fromCache(
      glow::ShaderType::VERTEX_SHADER, "fastcd/shaders/project_mesh.vert"));
  program_.attach(glow::GlShader::fromCache(
      glow::ShaderType::FRAGMENT_SHADER, "fastcd/shaders/project_mesh.frag"));
  program_.link();
}

/** \brief render given mesh from given view relative to the mesh. **/
void DepthProjector::render(const Mesh& mesh, const Eigen::Matrix4f& view) {
  GLint ov[4];
  glGetIntegerv(GL_VIEWPORT, ov);

  framebuffer_.bind();
  glEnable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0, 0, framebuffer_.width(), framebuffer_.height());

  program_.bind();
  program_.setUniform(
      glow::GlUniform<Eigen::Matrix4f>("mvp", projection_ * view));

  mesh.draw(program_);
  program_.release();

  framebuffer_.release();

  glViewport(ov[0], ov[1], ov[2], ov[3]);
}

/** \brief get depth projection. **/
glow::GlTexture& DepthProjector::texture() { return output_; }

}  // namespace fastcd
