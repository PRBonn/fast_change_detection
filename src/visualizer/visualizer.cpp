// Copyright 2017 Jens Behley (jens.behley@igg.uni-bonn.de), Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de), , Cyrill Stachniss, University of Bonn
#include "visualizer.h"

#include <glow/glutil.h>
#include <QtGui/QMouseEvent>
#include <glow/ScopedBinder.h>
#include "fastcd/camera.h"

namespace fastcd {

Visualizer::Visualizer(QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags f)
    : QGLWidget(parent, shareWidget, f), contextInitialized_(initContext()) {
  if (!contextInitialized_) throw std::runtime_error("OpenGL context initialization failed!");
  setFocusPolicy(Qt::StrongFocus);
  makeCurrent();
  prgDrawMesh_.attach(glow::GlShader::fromCache(
      glow::ShaderType::VERTEX_SHADER, "visualizer/shaders/draw_mesh.vert"));
  prgDrawMesh_.attach(glow::GlShader::fromCache(
      glow::ShaderType::FRAGMENT_SHADER, "visualizer/shaders/draw_mesh.frag"));
  prgDrawMesh_.link();

  // sun comes roughly from above...
  prgDrawMesh_.setUniform(glow::GlUniform<glow::vec4>("lights[0].position", glow::vec4(0, 0, -1, 0)));
  prgDrawMesh_.setUniform(glow::GlUniform<glow::vec3>("lights[0].ambient", glow::vec3(.9, .9, .9)));
  prgDrawMesh_.setUniform(glow::GlUniform<glow::vec3>("lights[0].diffuse", glow::vec3(.9, .9, .9)));
  prgDrawMesh_.setUniform(glow::GlUniform<glow::vec3>("lights[0].specular", glow::vec3(.9, .9, .9)));

  // more evenly distributed sun light...
  std::vector<glow::vec4> dirs = {glow::vec4(1, -1, 1, 0), glow::vec4(-1, -1, 1, 0), glow::vec4(1, -1, -1, 0), glow::vec4(-1, -1, -1, 0)};
  glow::vec3 indirect_intensity = glow::vec3(.1, .1, .1);

  for (uint32_t i = 0; i < 4; ++i) {
    std::stringstream light_name;
    light_name << "lights[" << (i + 1) << "]";

    prgDrawMesh_.setUniform(glow::GlUniform<glow::vec4>(light_name.str() + ".position", dirs[i]));
    prgDrawMesh_.setUniform(glow::GlUniform<glow::vec3>(light_name.str() + ".ambient", indirect_intensity));
    prgDrawMesh_.setUniform(glow::GlUniform<glow::vec3>(light_name.str() + ".diffuse", indirect_intensity));
    prgDrawMesh_.setUniform(glow::GlUniform<glow::vec3>(light_name.str() + ".specular", indirect_intensity));
  }

  prgDrawMesh_.setUniform(glow::GlUniform<int>("num_lights", 5));

  prgDrawMesh_.setUniform(glow::GlUniform<Eigen::Matrix4f>("model_mat", Eigen::Matrix4f::Identity()));
  prgDrawMesh_.setUniform(glow::GlUniform<Eigen::Matrix4f>("normal_mat", Eigen::Matrix4f::Identity()));

  connect(&timer_, SIGNAL(timeout()), this, SLOT(updateGL()));

  coloredPointProgram_.attach(
      glow::GlShader::fromCache(glow::ShaderType::VERTEX_SHADER,
                                "visualizer/shaders/colored_vertices.vert"));
  coloredPointProgram_.attach(
      glow::GlShader::fromCache(glow::ShaderType::FRAGMENT_SHADER,
                                "visualizer/shaders/colored_vertices.frag"));
  coloredPointProgram_.link();

  {
    // initialize coordinate axis buffer
    coordAxisVAO_.bind();

    float red = rgb2float(1.0f, 0.0f, 0.0f);
    float green = rgb2float(0.0f, 1.0f, 0.0f);
    float blue = rgb2float(0.0f, 0.0f, 1.0f);

    std::vector<float> cverts(
        {0, 0, 0, red, 1, 0, 0, red, 0, 0, 0, green, 0, 1, 0, green, 0, 0, 0, blue, 0, 0, 1, blue});
    coordAxisVBO_.assign(cverts);
    coordAxisVAO_.setVertexAttribute(0, coordAxisVBO_, 4, glow::AttributeType::FLOAT, false, 4 * sizeof(GLfloat), 0);
    coordAxisVAO_.enableVertexAttribute(0);

    coordAxisVAO_.release();
  }

  renderPoints_.attach(
      glow::GlShader::fromCache(glow::ShaderType::VERTEX_SHADER,
                                "visualizer/shaders/render_points.vert"));
  renderPoints_.attach(
      glow::GlShader::fromCache(glow::ShaderType::FRAGMENT_SHADER,
                                "visualizer/shaders/render_points.frag"));
  renderPoints_.link();

  renderPoints_.setUniform(glow::GlUniform<int32_t>("texOutput", 0));  // set to texture 0.

  sampler_.setMagnifyingOperation(glow::TexMagOp::NEAREST);
  sampler_.setMinifyingOperation(glow::TexMinOp::NEAREST);
  sampler_.setWrapOperation(glow::TexWrapOp::CLAMP_TO_BORDER, glow::TexWrapOp::CLAMP_TO_BORDER);
  timer_.start(1. / 30.);
}

float Visualizer::rgb2float(float r, float g, float b) {
  int32_t rgb = int32_t(round(r * 255.0f));
  rgb = (rgb << 8) + int32_t(round(g * 255.0f));
  rgb = (rgb << 8) + int32_t(round(b * 255.0f));

  return float(rgb);
}

void Visualizer::initializeGL() {
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_LINE_SMOOTH);

  camera_.lookAt(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
}

void Visualizer::resizeGL(int w, int h) {
  glViewport(0, 0, w, h);
  // set projection matrix
  float fov = glow::radians(45.0f);
  float aspect = float(w) / float(h);

  projection_ = glow::glPerspective(fov, aspect, 0.1f, 1000.0f);
}

glow::GlCamera::KeyboardModifier Visualizer::resolveKeyboardModifier(Qt::KeyboardModifiers modifiers) {
  // currently only single button presses are supported.
  glow::GlCamera::KeyboardModifier modifier = glow::GlCamera::KeyboardModifier::None;

  if (modifiers & Qt::ControlModifier)
    modifier = glow::GlCamera::KeyboardModifier::CtrlDown;
  else if (modifiers & Qt::ShiftModifier)
    modifier = glow::GlCamera::KeyboardModifier::ShiftDown;
  else if (modifiers & Qt::AltModifier)
    modifier = glow::GlCamera::KeyboardModifier::AltDown;

  return modifier;
}

glow::GlCamera::MouseButton Visualizer::resolveMouseButton(Qt::MouseButtons button) {
  // currently only single button presses are supported.
  glow::GlCamera::MouseButton btn = glow::GlCamera::MouseButton::NoButton;

  if (button & Qt::LeftButton)
    btn = glow::GlCamera::MouseButton::LeftButton;
  else if (button & Qt::RightButton)
    btn = glow::GlCamera::MouseButton::RightButton;
  else if (button & Qt::MiddleButton)
    btn = glow::GlCamera::MouseButton::MiddleButton;

  return btn;
}

glow::GlCamera::KeyboardKey Visualizer::resolveKeyboardKey(Qt::Key key) {
  switch(key) {
    case Qt::Key::Key_W: return glow::GlCamera::KeyboardKey::KeyW;
    case Qt::Key::Key_A: return glow::GlCamera::KeyboardKey::KeyA;
    case Qt::Key::Key_S: return glow::GlCamera::KeyboardKey::KeyS;
    case Qt::Key::Key_D: return glow::GlCamera::KeyboardKey::KeyD;
    case Qt::Key::Key_C: return glow::GlCamera::KeyboardKey::KeyC;
    case Qt::Key::Key_Space: return glow::GlCamera::KeyboardKey::KeySpace;
    default: return glow::GlCamera::KeyboardKey::KeyNotSupported;
  }
}

void Visualizer::mousePressEvent(QMouseEvent* event) {
  // if camera consumes the signal, simply return. // here we could also include some remapping.
  if (camera_.mousePressed(event->windowPos().x(), event->windowPos().y(), resolveMouseButton(event->buttons()),
                           resolveKeyboardModifier(event->modifiers()))) {
    return;
  }
}

void Visualizer::mouseReleaseEvent(QMouseEvent* event) {
  // if camera consumes the signal, simply return. // here we could also include some remapping.
  if (camera_.mouseReleased(event->windowPos().x(), event->windowPos().y(), resolveMouseButton(event->buttons()),
                            resolveKeyboardModifier(event->modifiers()))) {
    return;
  }
}

void Visualizer::mouseMoveEvent(QMouseEvent* event) {
  // if camera consumes the signal, simply return. // here we could also include some remapping.
  if (camera_.mouseMoved(event->windowPos().x(), event->windowPos().y(), resolveMouseButton(event->buttons()),
                         resolveKeyboardModifier(event->modifiers())))
    return;
}

void Visualizer::keyPressEvent(QKeyEvent* event) {
  try {
    int idx = std::stoi(event->text().toStdString());
    if (idx == 0) idx = 10;
    if (idx - 1 < cameras_.size()) {
      Eigen::Matrix3f rotation = cameras_[idx - 1]
                                     .GetPose()
                                     .cast<float>()
                                     .block<3, 3>(0, 0)
                                     .transpose();
      Eigen::Vector3f forward = Eigen::Vector3f(0, 0, 1);
      forward = (rotation * forward).normalized();
      forward(0) -= cameras_[idx - 1].GetPosition().cast<float>()(0);
      forward(1) -= cameras_[idx - 1].GetPosition().cast<float>()(1);
      forward(2) += cameras_[idx - 1].GetPosition().cast<float>()(2);

      camera_.lookAt(-cameras_[idx - 1].GetPosition()(0),
                     -cameras_[idx - 1].GetPosition()(1),
                     cameras_[idx - 1].GetPosition()(2), forward(0), forward(1),
                     forward(2));
    }
  } catch (...) {
  }
  if (camera_.keyPressed(resolveKeyboardKey(static_cast<Qt::Key>(event->key())),
                         resolveKeyboardModifier(event->modifiers()))) {
    return;
  }
}

void Visualizer::keyReleaseEvent(QKeyEvent* event) {
  if (camera_.keyReleased(
          resolveKeyboardKey(static_cast<Qt::Key>(event->key())),
          resolveKeyboardModifier(event->modifiers()))) {
    return;
  }
}

bool Visualizer::initContext() {
  // enabling core profile
  QGLFormat corefmt;
  corefmt.setVersion(5, 0);  // getting highest compatible format...
  corefmt.setProfile(QGLFormat::CoreProfile);
  setFormat(corefmt);

  // version info.
  QGLFormat fmt = this->format();
  std::cout << "OpenGL Context Version " << fmt.majorVersion() << "." << fmt.minorVersion() << " "
            << ((fmt.profile() == QGLFormat::CoreProfile) ? "core profile" : "compatibility profile") << std::endl;

  makeCurrent();
  glow::inititializeGLEW();

  return true;
}
void Visualizer::paintGL() {
  makeCurrent();

  glEnable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(bg_color_[0], bg_color_[1], bg_color_[2], 1.0f);
  view_ = camera_.matrix();
  //gl to model coordinates change
  view_ = view_ * glow::glRotateZ(M_PI);
  //gl to ScanNet dataset coordinates change
  // view_ = view_ * glow::glRotateX(-M_PI/2);
  mvp_ = projection_ * view_ * model_;
  if (show_axis_) {
    glow::ScopedBinder<glow::GlVertexArray> vao_binder(coordAxisVAO_);
    glow::ScopedBinder<glow::GlProgram> program_binder(coloredPointProgram_);

    coloredPointProgram_.setUniform(mvp_);

    glDrawArrays(GL_LINES, 0, 6);
  }

  //lines drawing
  if(!linesVAOs_.empty()){
    for (auto cameraVAO : linesVAOs_) {
      glow::ScopedBinder<glow::GlVertexArray> vao_binder(cameraVAO);
      glow::ScopedBinder<glow::GlProgram> program_binder(coloredPointProgram_);

      coloredPointProgram_.setUniform(mvp_);

      glDrawArrays(GL_LINES, 0, 8);
    }
    
  }
  prgDrawMesh_.bind();
  prgDrawMesh_.setUniform(mvp_);
  prgDrawMesh_.setUniform(glow::GlUniform<Eigen::Vector4f>("view_pos", camera_.getPosition()));
  for (auto mesh : meshes_) {
    mesh.draw(prgDrawMesh_);
  }
  prgDrawMesh_.release();

  if (show_output_) {
    vao_img_coords_.bind();
    renderPoints_.bind();
    renderPoints_.setUniform(mvp_);

    glActiveTexture(GL_TEXTURE0);
    output_.bind();
    sampler_.bind(0);  // ensure nearest neighbor interp.

    glDrawArrays(GL_POINTS, 0, output_.width() * output_.height());

    output_.release();
    sampler_.release(0);
    renderPoints_.release();
    vao_img_coords_.release();
  }
}

void Visualizer::ShowAxis(bool option) { show_axis_ = option; }

void Visualizer::addMesh(Mesh& mesh) {
  meshes_.push_back(mesh);
}

void Visualizer::setBackgroundColor(float r, float g, float b, float a){
  bg_color_ = {r, g, b, a};
}

void Visualizer::addCamera(Camera camera, float color, float length) {
  cameras_.push_back(camera);
  std::vector<float> vertices=camera.GetGlFovVertices(color, length);
  glow::GlVertexArray cameraVAO;
  glow::GlBuffer<float> cameraVBO{glow::BufferTarget::ARRAY_BUFFER,
                                    glow::BufferUsage::STATIC_DRAW};
  cameraVAO.bind();
  cameraVBO.assign(vertices);
  cameraVAO.setVertexAttribute(0, cameraVBO, 4, glow::AttributeType::FLOAT, false, 4 * sizeof(GLfloat), 0);
  cameraVAO.enableVertexAttribute(0);
  cameraVAO.release();
  linesVAOs_.push_back(cameraVAO);
  linesVBOs_.push_back(cameraVBO);
}

void Visualizer::addPoint(const Eigen::Vector3f& position, float size,
                      float color) {
  std::vector<float> vertices({position(0) + size / 2,
                            position(1),
                            position(2),
                            color,

                            position(0) - size / 2,
                            position(1),
                            position(2),
                            color,

                            position(0),
                            position(1),
                            position(2),
                            color,

                            position(0),
                            position(1) + size / 2,
                            position(2),
                            color,

                            position(0),
                            position(1) - size / 2,
                            position(2),
                            color,

                            position(0),
                            position(1),
                            position(2),
                            color,

                            position(0),
                            position(1),
                            position(2) + size / 2,
                            color,

                            position(0),
                            position(1),
                            position(2) - size / 2,
                            color});

  glow::GlVertexArray cameraVAO;
  glow::GlBuffer<float> cameraVBO{glow::BufferTarget::ARRAY_BUFFER,
                                    glow::BufferUsage::STATIC_DRAW};
  cameraVAO.bind();
  cameraVBO.assign(vertices);
  cameraVAO.setVertexAttribute(0, cameraVBO, 4, glow::AttributeType::FLOAT, false, 4 * sizeof(GLfloat), 0);
  cameraVAO.enableVertexAttribute(0);
  cameraVAO.release();
  linesVAOs_.push_back(cameraVAO);
  linesVBOs_.push_back(cameraVBO);
}

void Visualizer::addLine(const Eigen::Vector3f& start, const Eigen::Vector3f& end,
               float color) {
  std::vector<float> vertices({start(0), start(1), start(2), color,
                               end(0), end(1), end(2), color});
  glow::GlVertexArray cameraVAO;
  glow::GlBuffer<float> cameraVBO{glow::BufferTarget::ARRAY_BUFFER,
                                    glow::BufferUsage::STATIC_DRAW};
  cameraVAO.bind();
  cameraVBO.assign(vertices);
  cameraVAO.setVertexAttribute(0, cameraVBO, 4, glow::AttributeType::FLOAT, false,
                               4 * sizeof(GLfloat), 0);
  cameraVAO.enableVertexAttribute(0);
  cameraVAO.release();
  linesVAOs_.push_back(cameraVAO);
  linesVBOs_.push_back(cameraVBO);
}


void Visualizer::setOutput(glow::GlTexture& texture) {
  std::cout << "Should render now points on the mesh surface!" << std::endl;
  show_output_ = true;
  output_ = texture;
  uint32_t width = texture.width();
  uint32_t height = texture.height();

  // initialize vbo for image coordinates. (needed for the vertex shader)
  std::vector<glow::vec2> img_coords;
  img_coords.reserve(width * height);
  float inv_width = 1.0f / float(width);
  float inv_height = 1.0f / float(height);

  // for each pixel of the texture, we are generating a "virtual vertex"
  for (uint32_t x = 0; x < width; ++x) {
    for (uint32_t y = 0; y < height; ++y) {
      img_coords.push_back(glow::vec2(inv_width * float(x + 0.5f), inv_height * float(y + 0.5f)));
    }
  }

  vbo_img_coords_.assign(img_coords);
  vao_img_coords_.bind();
  vao_img_coords_.setVertexAttribute(0, vbo_img_coords_, 2, glow::AttributeType::FLOAT, false, 2 * sizeof(float),
                                     0);  // Note: does also the binding of the vbo!
  vao_img_coords_.enableVertexAttribute(0);
  vao_img_coords_.release();

  updateGL();
}

}  // namespace fastcd
