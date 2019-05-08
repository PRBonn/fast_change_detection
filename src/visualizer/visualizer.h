// Copyright 2017 Jens Behley (jens.behley@igg.uni-bonn.de), Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de), , Cyrill Stachniss, University of Bonn
#pragma once

#include <vector>

#include <glow/glbase.h>

#include <QtCore/QTimer>
#include <QtOpenGL/QGLWidget>

#include <glow/GlUniform.h>
#include <glow/util/FpsCamera.h>
#include <glow/GlTexture.h>
#include <glow/GlSampler.h>
#include "fastcd/mesh.h"
#include "fastcd/camera.h"

namespace fastcd {

/**
 * @brief      Class for an OpenGL visualization.
 */
class Visualizer : public QGLWidget {
  Q_OBJECT

 public:
  /**
   * @brief      Constructor.
   */
  Visualizer(QWidget* parent = 0, const QGLWidget* shareWidget = 0,
         Qt::WindowFlags f = 0);

  /**
   * @brief      Adds a mesh to the 3D world.
   *
   * @param      mesh  The mesh
   */
  void addMesh(Mesh& mesh);

  /**
   * @brief      Sets the background color.
   *
   * @param[in]  r     The red value
   * @param[in]  g     The green value
   * @param[in]  b     The blue value
   * @param[in]  a     The alpha value
   */
  void setBackgroundColor(float r, float g, float b, float a);

  /**
   * @brief      Sets the texture to be painted on the 3D model. (for debug
   *             purpose)
   *
   * @param      texture  The texture
   */
  void setOutput(glow::GlTexture& texture);

  /**
   * @brief      Adds a camera do be drawn in the 3D world. Its position is also
   *             a teleport point of the user camera.
   *
   * @param[in]  camera  The camera
   * @param[in]  color   The color of the visualized frustum
   * @param[in]  length  The length of the visualized frustum
   */
  void addCamera(Camera camera, float color, float length);

  /**
   * @brief      Adds a point to be drawn in the 3D world (as a cross).
   *
   * @param[in]  position  The position of the point
   * @param[in]  size      The size of the point
   * @param[in]  color     The color of the point
   */
  void addPoint(const Eigen::Vector3f& position, float size, float color);

  /**
   * @brief      Adds a line to be drawn in the 3D world.
   *
   * @param[in]  start  The starting point of the line
   * @param[in]  end    The ending point of the line
   * @param[in]  color  The color of the line
   */
  void addLine(const Eigen::Vector3f& start, const Eigen::Vector3f& end,
               float color);

  /**
   * @brief      Helper fuction that packs the red, green and blue values of a
   *             color in a single float.
   *
   * @param[in]  r     The red value
   * @param[in]  g     The green value
   * @param[in]  b     The blue value
   *
   * @return     The color packed in a single float.
   */
  static float rgb2float(float r, float g, float b);

  void ShowAxis(bool option);

 protected:
  bool initContext();

  void initializeGL();
  void resizeGL(int w, int h);
  void paintGL();

  void mousePressEvent(QMouseEvent* event);
  void mouseReleaseEvent(QMouseEvent* event);
  void mouseMoveEvent(QMouseEvent* event);
  void keyPressEvent(QKeyEvent* event);
  void keyReleaseEvent(QKeyEvent* event);

  glow::GlCamera::KeyboardModifier resolveKeyboardModifier(
      Qt::KeyboardModifiers modifiers);
  glow::GlCamera::MouseButton resolveMouseButton(Qt::MouseButtons button);
  glow::GlCamera::KeyboardKey resolveKeyboardKey(Qt::Key key);
  // hack to have GLEW initalized before anything else.
  bool contextInitialized_;

  glow::GlUniform<Eigen::Matrix4f> mvp_{"mvp", Eigen::Matrix4f::Identity()};
  glow::GlUniform<Eigen::Matrix4f> mvp_inv_t_{"mvp_inv_t",
                                              Eigen::Matrix4f::Identity()};

  QTimer timer_;

  glow::FpsCamera camera_;

  Eigen::Matrix4f model_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f view_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f projection_{Eigen::Matrix4f::Identity()};

  std::vector<Mesh> meshes_;
  glow::GlProgram prgDrawMesh_;
  glow::GlProgram coloredPointProgram_;

  glow::GlBuffer<float> coordAxisVBO_{glow::BufferTarget::ARRAY_BUFFER,
                                    glow::BufferUsage::STATIC_DRAW};  // x, y, z, color(float)
  glow::GlVertexArray coordAxisVAO_;
  std::vector<glow::GlBuffer<float>> linesVBOs_;
  std::vector<glow::GlVertexArray> linesVAOs_;

  // visualization of the image points.
  bool show_output_{false};
  glow::GlTexture output_{10, 10, glow::TextureFormat::RGBA_FLOAT};
  glow::GlVertexArray vao_img_coords_;
  glow::GlBuffer<glow::vec2> vbo_img_coords_{glow::BufferTarget::ARRAY_BUFFER,
                                             glow::BufferUsage::STATIC_DRAW};
  glow::GlSampler sampler_;
  glow::GlProgram renderPoints_;

  std::vector<float> bg_color_{0.0f, 0.0f, 0.0f, 1.0f};
  std::vector<Camera> cameras_;

  bool show_axis_{true};
};

}  // namespace fastcd
