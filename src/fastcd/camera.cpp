// Copyright 2017 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de), Cyrill Stachniss, University of Bonn
#include "fastcd/camera.h"
#include <glow/glutil.h>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

namespace fastcd {

void Camera::ReadCalibration(const std::string& filename, int id) {
  boost::property_tree::ptree pt, resolution, camera;
  read_xml(filename, pt);

  double cx = pt.get<double>("document.chunk.sensors.sensor.calibration.cx");
  double cy = pt.get<double>("document.chunk.sensors.sensor.calibration.cy");
  double fx = pt.get<double>("document.chunk.sensors.sensor.calibration.fx");
  double fy = pt.get<double>("document.chunk.sensors.sensor.calibration.fy");
  resolution =
      pt.get_child("document.chunk.sensors.sensor.calibration.resolution");
  width_ = resolution.get<int>("<xmlattr>.width");
  height_ = resolution.get<int>("<xmlattr>.height");

  BOOST_FOREACH(boost::property_tree::ptree::value_type const& v,
                 pt.get_child("document.chunk.cameras")) {
    if (v.first == "camera" && v.second.get<int>("<xmlattr>.id") == id) {
      std::string s = v.second.get<std::string>("transform");
      std::stringstream sstream(s);
      for (size_t i = 0; i < 4; i++)
        for (size_t j = 0; j < 4; j++) sstream >> pose_(i, j);
    }
  }

  calibration_.setIdentity();
  calibration_(0, 0) = fx;
  calibration_(1, 1) = fy;
  calibration_(0, 2) = cx;
  calibration_(1, 2) = cy;
  inverse_calibration_ = calibration_.inverse();
}

void Camera::ScaleCalibration(double scaling) {
  Eigen::Matrix3d k_transform;
  k_transform << scaling, 0, scaling/2-0.5,
                 0, scaling, scaling/2-0.5,
                 0,       0,             1;
  width_ *= scaling;
  height_ *= scaling;
  calibration_ = k_transform * calibration_;
  inverse_calibration_ = calibration_.inverse();
}


Eigen::Vector2i Camera::Project(double x, double y, double z) const {
  Eigen::Vector4d world_point;
  Eigen::Vector3d camera_point;
  Eigen::Matrix<double, 3, 4> P;
  Eigen::Matrix3d R = pose_.block<3, 3>(0, 0).transpose();
  world_point << x, y, z, 1;
  P.block<3, 3>(0, 0) = calibration_ * R;
  P.block<3, 1>(0, 3) = -calibration_ * R * pose_.block<3, 1>(0, 3);
  camera_point = P * world_point;
  Eigen::Vector2i uv_coord;
  uv_coord << static_cast<int>(camera_point(0) / camera_point(2)),
      static_cast<int>(camera_point(1) / camera_point(2));
  return uv_coord;
}

Eigen::Vector2i Camera::Project(const Eigen::Vector3d& point) const {
  return Project(point(0), point(1), point(2));
}

Eigen::Vector3d Camera::BackProject(int u, int v) {
  Eigen::Vector3d camera_point;
  camera_point << u, v, 1;
  Eigen::Vector4d world_point, hom_direction;
  Eigen::Vector3d direction;
  direction = inverse_calibration_ * camera_point;
  hom_direction << direction(0), direction(1), direction(2), 1;
  world_point = pose_ * hom_direction;
  direction << world_point(0) / world_point(3), world_point(1) / world_point(3),
      world_point(2) / world_point(3);
  return direction;
}

Eigen::Vector3d Camera::BackProject(const Eigen::Vector2i& point) {
  return BackProject(point(0), point(1));
}

std::vector<float> Camera::GetGlFovVertices(float color, float length) {
  Eigen::Vector3f position = GetPosition().cast<float>();

  Eigen::Vector3f direction_tl = BackProject(0, 0).cast<float>();
  Eigen::Vector3f direction_bl = BackProject(0, height_).cast<float>();
  Eigen::Vector3f direction_tr = BackProject(width_, 0).cast<float>();
  Eigen::Vector3f direction_br = BackProject(width_, height_).cast<float>();
  Eigen::Vector3f corner_tl = (direction_tl - position) * length + position;
  Eigen::Vector3f corner_bl = (direction_bl - position) * length + position;
  Eigen::Vector3f corner_tr = (direction_tr - position) * length + position;
  Eigen::Vector3f corner_br = (direction_br - position) * length + position;

  std::vector<float> verts({position(0),  position(1),  position(2),  color,
                            corner_tl(0), corner_tl(1), corner_tl(2), color,
                            position(0),  position(1),  position(2),  color,
                            corner_bl(0), corner_bl(1), corner_bl(2), color,
                            position(0),  position(1),  position(2),  color,
                            corner_tr(0), corner_tr(1), corner_tr(2), color,
                            position(0),  position(1),  position(2),  color,
                            corner_br(0), corner_br(1), corner_br(2), color});
  return verts;
}

std::ostream& operator<<(std::ostream& os, const Camera& c) {
  os << "Resolution: " << c.width_ << "x" << c.height_ << std::endl;
  os << "K:\n" << c.calibration_ << std::endl;
  os << "\nPose:\n" << c.pose_ << std::endl;
  return os;
}

Eigen::Matrix3d Camera::GetK() const { return calibration_; }

Eigen::Matrix3d Camera::GetInvK() const { return inverse_calibration_; }

Eigen::Matrix4d Camera::GetPose() const { return pose_; }

Eigen::Vector3d Camera::GetPosition() const {
  return pose_.block<3, 1>(0, 3);
}

Eigen::Matrix<double, 3, 4> Camera::GetP() const {
  Eigen::Matrix<double, 3, 4> P;
  Eigen::Matrix3d R = pose_.block<3, 3>(0, 0).transpose();
  P.block<3, 3>(0, 0) = calibration_ * R;
  P.block<3, 1>(0, 3) = -calibration_ * R * pose_.block<3, 1>(0, 3);
  return P;
}

Eigen::Matrix4f Camera::GetGlProjection(float near, float far) {
  float left = 0.0f;
  float right = static_cast<float>(width_);
  float bottom = 0.0f;
  float top = static_cast<float>(height_);
  Eigen::Matrix4f gl_ortho, gl_persp, gl_camera;
  gl_persp.setZero();
  gl_persp.block<3, 3>(0, 0) = calibration_.cast<float>();
  gl_persp.block<2, 1>(0, 2) *= -1;
  gl_persp(2, 2) = near + far;
  gl_persp(2, 3) = near * far;
  gl_persp(3, 2) = -1;

  gl_ortho << 2 / (right - left), 0, 0, -(right + left) / (right - left),
              0, 2 / (top - bottom), 0, -(top + bottom) / (top - bottom),
              0, 0, -2 / (far - near), -(far + near) / (far - near),
              0, 0, 0, 1;

  gl_camera = gl_ortho * gl_persp;

  return gl_camera;
}

Eigen::Matrix4f Camera::GetGlView() {
  return glow::glRotateY(M_PI) * glow::glRotateZ(M_PI) *
         GetPose().inverse().cast<float>();
}

int Camera::GetWidth() { return width_; }

int Camera::GetHeight() { return height_; }

}  // namespace fastcd
