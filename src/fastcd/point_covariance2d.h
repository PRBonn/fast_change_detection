// Copyright 2017 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de)
#pragma once
#include <vector>
#include <Eigen/Core>

namespace fastcd {

/**
 * @brief      Class for storing a 2D point, its covariance and the
 *             corresponding confidence ellipse.          
 */
class PointCovariance2d {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief      Empty constructor.
   */
  PointCovariance2d();

  /**
   * @brief      Constructor. It computes all the parameters of the ellipse from
   *             point, covariance and chi square;
   *
   * @param[in]  point        The point
   * @param[in]  covariance  The covariance
   * @param[in]  chi_square  The chi square. 95% of the variance retained by
   *                         default
   */
  PointCovariance2d(const Eigen::Vector2d &point,
                    const Eigen::Matrix2d &covariance,
                    double chi_square = 5.991);

  /**
   * @brief      Gets the point.
   *
   * @return     The point.
   */
  Eigen::Vector2d Point();

  /**
   * @brief      Gets the covariance matrix.
   *
   * @return     The covariance matrix.
   */
  Eigen::Matrix2d Covariance();

  /**
   * @brief      Gets the eigenvalues of the covariance matrix.
   *
   * @return     The eigenvalues of the covariance matrix.
   */
  Eigen::Vector2d Eigenvalues();

  /**
   * @brief      Gets the eigenvectors of the covariance matrix.
   *
   * @return     The eigenvectors of the covariance matrix.
   */
  Eigen::Matrix2d Eigenvectors();

  /**
   * @brief      Gets the angle of the confidence ellipse w.r.t. the x-axis.
   *
   * @return     The angle of the confidence ellipse w.r.t. the x-axis.
   */
  double Angle();

  /**
   * @brief      Gets the length of the major axis of the confidence ellipse.
   *
   * @return     The length of the major axis of the confidence ellipse.
   */
  double HalfMajorAxisSize();

  /**
   * @brief      Gets the length of the minor axis of the confidence ellipse.
   *
   * @return     The length of the minor axis of the confidence ellipse.
   */
  double HalfMinorAxisSize();

  /**
   * @brief      Gets the vertices of the confidence ellipse.
   *
   * @return     The 4 vertices of the confidence ellipse.
   */
  std::vector<Eigen::Vector2d> Vertices();

 protected:
  /** The point */
  Eigen::Vector2d point_;

  /** The covariance matrix */
  Eigen::Matrix2d covariance_;

  /** The eigenvalues of the covariance matrix */
  Eigen::Vector2d eigenvalues_;

  /** The eigenvectors of the covariance matrix */
  Eigen::Matrix2d eigenvectors_;

  /** The angle of the confidence ellipse w.r.t. the x-axis */
  double angle_;

  /** The length of the major axis of the confidence ellipse */
  double half_major_axis_size_;

  /** The length of the minor axis of the confidence ellipse */
  double half_minor_axis_size_;

  /** The 4 vertices of the confidence ellipse */
  std::vector<Eigen::Vector2d> vertices_;
};

}  // namespace fastcd
