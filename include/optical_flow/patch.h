/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once

#include <Eigen/Dense>
#include <sophus/se2.hpp>

#include "../image/image.h"
#include "patterns.h"

namespace basalt {

template <typename Scalar, typename Pattern>
struct OpticalFlowPatch {
  static constexpr int PATTERN_SIZE = Pattern::PATTERN_SIZE;

  typedef Eigen::Matrix<int, 2, 1> Vector2i;

  typedef Eigen::Matrix<Scalar, 2, 1> Vector2;
  typedef Eigen::Matrix<Scalar, 1, 2> Vector2T;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
  typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
  typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;
  typedef Eigen::Matrix<Scalar, PATTERN_SIZE, 1> VectorP;

  typedef Eigen::Matrix<Scalar, 2, PATTERN_SIZE> Matrix2P;
  typedef Eigen::Matrix<Scalar, PATTERN_SIZE, 2> MatrixP2;
  typedef Eigen::Matrix<Scalar, PATTERN_SIZE, 3> MatrixP3;
  typedef Eigen::Matrix<Scalar, 3, PATTERN_SIZE> Matrix3P;
  typedef Eigen::Matrix<Scalar, PATTERN_SIZE, 4> MatrixP4;
  typedef Eigen::Matrix<int, 2, PATTERN_SIZE> Matrix2Pi;

  static const Matrix2P pattern2;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OpticalFlowPatch() = default;

  OpticalFlowPatch(const Image<const uint16_t> &img, const Vector2 &pos) {
    setFromImage(img, pos);
  }

  template <typename ImgT>
  static void setData(const ImgT &img, const Vector2 &pos, Scalar &mean,
                      VectorP &data, const Sophus::SE2<Scalar> *se2 = nullptr) {
    int num_valid_points = 0;
    Scalar sum = 0;

    for (int i = 0; i < PATTERN_SIZE; i++) {
      Vector2 p;
      if (se2) {
        p = pos + (*se2) * pattern2.col(i);
      } else {
        p = pos + pattern2.col(i);
      };

      if (img.InBounds(p, 2)) {
        Scalar val = img.template interp<Scalar>(p);
        data[i] = val;
        sum += val;
        num_valid_points++;
      } else {
        data[i] = -1;
      }
    }

    mean = sum / num_valid_points;
    data /= mean;
  }

  template <typename ImgT>
  static void setDataJacSe2(const ImgT &img, const Vector2 &pos, Scalar &mean,
                            VectorP &data, MatrixP3 &J_se2) {
    int num_valid_points = 0;
    Scalar sum = 0;
    Vector3 grad_sum_se2(0, 0, 0);

    Eigen::Matrix<Scalar, 2, 3> Jw_se2;
    Jw_se2.template topLeftCorner<2, 2>().setIdentity();

    for (int i = 0; i < PATTERN_SIZE; i++) {
      Vector2 p = pos + pattern2.col(i);

      // Fill jacobians with respect to SE2 warp
      Jw_se2(0, 2) = -pattern2(1, i);
      Jw_se2(1, 2) = pattern2(0, i);

      if (img.InBounds(p, 2)) {
        Vector3 valGrad = img.template interpGrad<Scalar>(p);
        data[i] = valGrad[0];
        sum += valGrad[0];
        J_se2.row(i) = valGrad.template tail<2>().transpose() * Jw_se2;
        grad_sum_se2 += J_se2.row(i);
        num_valid_points++;
      } else {
        data[i] = -1;
      }
    }

    mean = sum / num_valid_points;

    const Scalar mean_inv = num_valid_points / sum;

    for (int i = 0; i < PATTERN_SIZE; i++) {
      if (data[i] >= 0) {
        J_se2.row(i) -= grad_sum_se2.transpose() * data[i] / sum;
        data[i] *= mean_inv;
      } else {
        J_se2.row(i).setZero();
      }
    }
    J_se2 *= mean_inv;
  }

  void setFromImage(const Image<const uint16_t> &img, const Vector2 &pos) {
    this->pos = pos;

    MatrixP3 J_se2;

    setDataJacSe2(img, pos, mean, data, J_se2);

    Matrix3 H_se2 = J_se2.transpose() * J_se2;
    Matrix3 H_se2_inv;
    H_se2_inv.setIdentity();
    H_se2.ldlt().solveInPlace(H_se2_inv);

    H_se2_inv_J_se2_T = H_se2_inv * J_se2.transpose();

    // NOTE: while it's very unlikely we get a source patch with all black
    // pixels, since points are usually selected at corners, it doesn't cost
    // much to be safe here.

    // all-black patch cannot be normalized; will result in mean of "zero" and
    // H_se2_inv_J_se2_T will contain "NaN" and data will contain "inf"
    valid = mean > std::numeric_limits<Scalar>::epsilon() &&
            H_se2_inv_J_se2_T.array().isFinite().all() &&
            data.array().isFinite().all();
  }

  inline bool residual(const Image<const uint16_t> &img,
                       const Matrix2P &transformed_pattern,
                       VectorP &residual) const {
    Scalar sum = 0;
    int num_valid_points = 0;

    for (int i = 0; i < PATTERN_SIZE; i++) {
      if (img.InBounds(transformed_pattern.col(i), 2)) {
        residual[i] = img.interp<Scalar>(transformed_pattern.col(i));
        sum += residual[i];
        num_valid_points++;
      } else {
        residual[i] = -1;
      }
    }

    // all-black patch cannot be normalized
    if (sum < std::numeric_limits<Scalar>::epsilon()) {
      residual.setZero();
      return false;
    }

    int num_residuals = 0;

    for (int i = 0; i < PATTERN_SIZE; i++) {
      if (residual[i] >= 0 && data[i] >= 0) {
        const Scalar val = residual[i];
        residual[i] = num_valid_points * val / sum - data[i];
        num_residuals++;

      } else {
        residual[i] = 0;
      }
    }

    return num_residuals > PATTERN_SIZE / 2;
  }

  Vector2 pos = Vector2::Zero();
  VectorP data = VectorP::Zero();  // negative if the point is not valid

  // MatrixP3 J_se2;  // total jacobian with respect to se2 warp
  // Matrix3 H_se2_inv;
  Matrix3P H_se2_inv_J_se2_T = Matrix3P::Zero();

  Scalar mean = 0;

  bool valid = false;
};

template <typename Scalar, typename Pattern>
const typename OpticalFlowPatch<Scalar, Pattern>::Matrix2P
    OpticalFlowPatch<Scalar, Pattern>::pattern2 = Pattern::pattern2;

}  // namespace basalt
