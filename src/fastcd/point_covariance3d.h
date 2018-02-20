// Copyright 2017 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de)
#pragma once
#include <vector>
#include "fastcd/mesh.h"

namespace fastcd {

/**
 * @brief      Class for storing a 3D point, its covariance and the
 *             corresponding confidence ellipsoid.         
 */
class PointCovariance3d {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief      Constructor. It computes all the parameters of the ellipsoid
   *             from point, covariance and chi square;
   *
   * @param[in]  point       The point
   * @param[in]  covariance  The covariance
   * @param[in]  chi_square  The chi square. 95% of the variance retained by
   *                         default
   */
  PointCovariance3d(const Eigen::Vector3d &point,
                    const Eigen::Matrix3d &covariance,
                    double chi_square = 7.815);

  /**
   * @brief      Creates and returns a Mesh representing the confidence
   *             ellipsoid.
   *
   * @param[in]  stacks  The number of stacks
   * @param[in]  slices  The number of slices
   * @param[in]  r       The red color value
   * @param[in]  g       The green color value
   * @param[in]  b       The blue color value
   * @param[in]  a       The alpha value
   *
   * @return     The mesh.
   */
  Mesh ToMesh(int stacks, int slices, float r, float g, float b, float a) const;

  /**
   * @brief      Gets the point.
   *
   * @return     The point.
   */
  Eigen::Vector3d Point() const;

  /**
   * @brief      Gets the covariance matrix.
   *
   * @return     The covariance matrix.
   */
  Eigen::Matrix3d Covariance() const;

  /**
   * @brief      Gets the eigenvalues of the covariance matrix.
   *
   * @return     The eigenvalues of the covariance matrix.
   */
  Eigen::Vector3d Eigenvalues() const;

  /**
   * @brief      Gets the eigenvectors of the covariance matrix.
   *
   * @return     The eigenvectors of the covariance matrix.
   */
  Eigen::Matrix3d Eigenvectors() const;

  /**
   * @brief      Gets the transformation matrix to scale the confidence
   *             ellipsoid.
   *
   * @return     The transformation matrix to scale the confidence ellipsoid.
   */
  Eigen::Matrix3d Scaling() const;

  /**
   * @brief      Gets the vertices of the confidence ellipsoid.
   *
   * @return     The 6 vertices of the confidence ellipsoid.
   */
  std::vector<Eigen::Vector3d> Vertices() const;

 protected:
  /** The point */
  Eigen::Vector3d point_;

  /** The covariance matrix */
  Eigen::Matrix3d covariance_;

  /** The eigenvalues of the covariance matrix */
  Eigen::Vector3d eigenvalues_;

  /** The eigenvectors of the covariance matrix */
  Eigen::Matrix3d eigenvectors_;

  /** The transformation matrix to scale the confidence ellipsoid */
  Eigen::Matrix3d scaling_;

  /** The 6 vertices of the confidence ellipsoid */
  std::vector<Eigen::Vector3d> vertices_;
};

}  // namespace fastcd
