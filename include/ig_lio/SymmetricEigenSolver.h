#ifndef SYMMETRIC_EIGEN_SOLVER_H_
#define SYMMETRIC_EIGEN_SOLVER_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

class SymmetricEigensolver3x3 {
 public:
  SymmetricEigensolver3x3();

  SymmetricEigensolver3x3(const Eigen::Matrix3d input_matrix);

  void compute();

  Eigen::Vector3d eigenvalues();

  Eigen::Matrix3d eigenvectors();

 private:
  void computeEigenvector0(double a00,
                           double a01,
                           double a02,
                           double a11,
                           double a12,
                           double a22,
                           int i0);

  void computeEigenvector1(double a00,
                           double a01,
                           double a02,
                           double a11,
                           double a12,
                           double a22,
                           int i0,
                           int i1);

  void computeEigenvector2(int i0, int i1, int i2);

  void computeOrthogonalComplement(Eigen::Vector3d& w,
                                   Eigen::Vector3d& u,
                                   Eigen::Vector3d& v);

  Eigen::Vector3d cross(Eigen::Vector3d u, Eigen::Vector3d v);

  Eigen::Matrix3d input_;
  Eigen::Matrix3d evecs_;
  Eigen::Vector3d evals_;
};

#endif
