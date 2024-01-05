#include "ig_lio/lio.h"
#include "ig_lio/timer.h"

extern Timer timer;

bool LIO::MeasurementUpdate(SensorMeasurement& sensor_measurement) {
  if (sensor_measurement.measurement_type_ == MeasurementType::LIDAR) {
    // range filter
    CloudPtr filtered_cloud_ptr(new CloudType());
    filtered_cloud_ptr->points.reserve(sensor_measurement.cloud_ptr_->size());
    for (const auto& pt : sensor_measurement.cloud_ptr_->points) {
      if (InRadius(pt)) {
        filtered_cloud_ptr->points.emplace_back(pt);
      }
    }
    sensor_measurement.cloud_ptr_ = filtered_cloud_ptr;

    timer.Evaluate(
        [&, this]() {
          // transform scan from lidar's frame to imu's frame
          CloudPtr cloud_body_ptr(new CloudType());
          pcl::transformPointCloud(*sensor_measurement.cloud_ptr_,
                                   *cloud_body_ptr,
                                   config_.T_imu_lidar);
          sensor_measurement.cloud_ptr_ = std::move(cloud_body_ptr);

          // undistort
          if (config_.enable_undistort) {
            UndistortPointCloud(sensor_measurement.bag_time_,
                                sensor_measurement.lidar_end_time_,
                                sensor_measurement.cloud_ptr_);
          }
        },
        "undistort");

    timer.Evaluate(
        [&, this]() {
          fast_voxel_grid_ptr_->Filter(
              sensor_measurement.cloud_ptr_, cloud_DS_ptr_, cloud_cov_ptr_);
        },
        "downsample");
  }

  // Make sure the local map is dense enought to measurement update
  if (lidar_frame_count_ <= 10) {
    CloudPtr trans_cloud_ptr(new CloudType());
    pcl::transformPointCloud(
        *sensor_measurement.cloud_ptr_, *trans_cloud_ptr, curr_state_.pose);
    voxel_map_ptr_->AddCloud(trans_cloud_ptr);
    lidar_frame_count_++;
    return true;
  }

  // measurement update
  prev_state_ = curr_state_;
  iter_num_ = 0;
  need_converge_ = false;
  Eigen::Matrix<double, 15, 1> delta_x = Eigen::Matrix<double, 15, 1>::Zero();
  while (iter_num_ < config_.max_iterations) {
    StepOptimize(sensor_measurement, delta_x);

    if (IsConverged(delta_x)) {
      // Optimization convergence, exit
      break;
    } else {
      // The first three iterations perform KNN, then no longer perform, thus
      // accelerating the problem convergence
      if (iter_num_ < 3) {
        need_converge_ = false;
      } else {
        need_converge_ = true;
      }
    }

    iter_num_++;
  }

  // LOG(INFO) << "final hessian: " << std::endl << final_hessian_;
  // P_ = final_hessian_.inverse();
  ComputeFinalCovariance(delta_x);
  prev_state_ = curr_state_;

  timer.Evaluate(
      [&, this]() {
        if (lidar_frame_count_ < 10) {
          CloudPtr trans_cloud_ptr(new CloudType());
          pcl::transformPointCloud(*sensor_measurement.cloud_ptr_,
                                   *trans_cloud_ptr,
                                   curr_state_.pose);
          voxel_map_ptr_->AddCloud(trans_cloud_ptr);

          last_keyframe_pose_ = curr_state_.pose;
        } else {
          Eigen::Matrix4d delta_p =
              last_keyframe_pose_.inverse() * curr_state_.pose;
          // The keyframe strategy ensures an appropriate spatial pattern of the
          // points in each voxel
          if (effect_feat_num_ < 1000 ||
              delta_p.block<3, 1>(0, 3).norm() > 0.5 ||
              Sophus::SO3d(delta_p.block<3, 3>(0, 0)).log().norm() > 0.18) {
            CloudPtr trans_cloud_DS_ptr(new CloudType());
            pcl::transformPointCloud(
                *cloud_DS_ptr_, *trans_cloud_DS_ptr, curr_state_.pose);
            voxel_map_ptr_->AddCloud(trans_cloud_DS_ptr);

            last_keyframe_pose_ = curr_state_.pose;
            keyframe_count_++;
          }
        }
      },
      "update voxel map");

  lidar_frame_count_++;

  ava_effect_feat_num_ += (effect_feat_num_ - ava_effect_feat_num_) /
                          static_cast<double>(lidar_frame_count_);
  LOG(INFO) << "curr_feat_num: " << effect_feat_num_
            << " ava_feat_num: " << ava_effect_feat_num_
            << " keyframe_count: " << keyframe_count_
            << " lidar_frame_count: " << lidar_frame_count_
            << " grid_size: " << voxel_map_ptr_->GetVoxelMapSize();
  return true;
}

bool LIO::StepOptimize(const SensorMeasurement& sensor_measurement,
                       Eigen::Matrix<double, 15, 1>& delta_x) {
  Eigen::Matrix<double, 15, 15> H = Eigen::Matrix<double, 15, 15>::Zero();
  Eigen::Matrix<double, 15, 1> b = Eigen::Matrix<double, 15, 1>::Zero();

  double y0 = 0;
  switch (sensor_measurement.measurement_type_) {
  case MeasurementType::LIDAR: {
    double y0_lidar = 0.0;

    timer.Evaluate(
        [&, this]() {
          // After LIO has moved some distance, each voxel is already well
          // formulate
          // the surrounding environments
          if (keyframe_count_ > 20) {
            y0_lidar = ConstructGICPConstraints(H, b);
          }
          // In the initial state, the probability of each voxel is poor
          // use point-to-plane instead of GICP
          else {
            y0_lidar = ConstructPoint2PlaneConstraints(H, b);
          }
        },
        "lidar constraints");

    y0 += y0_lidar;
    break;
  }

  default: {
    LOG(ERROR) << "error measurement type!";
    exit(0);
  }
  }

  // LOG(INFO) << "lidar H: " << std::endl << H << std::endl;

  timer.Evaluate(
      [&, this]() {
        double y0_imu = ConstructImuPriorConstraints(H, b);
        y0 += y0_imu;
      },
      "imu constraint");

  GNStep(sensor_measurement, H, b, y0, delta_x);

  return true;
}

bool LIO::GNStep(const SensorMeasurement& sensor_measurement,
                 Eigen::Matrix<double, 15, 15>& H,
                 Eigen::Matrix<double, 15, 1>& b,
                 const double y0,
                 Eigen::Matrix<double, 15, 1>& delta_x) {
  timer.Evaluate(
      [&, this]() {
        // The function inverse() has better numerical stability
        // And the dimension is small, direct inversion is not time-consuming
        Eigen::Matrix<double, 15, 1> dir = -H.inverse() * b;

        State new_state;
        delta_x = dir;
        CorrectState(curr_state_, delta_x, new_state);
        curr_state_ = new_state;

        final_hessian_ = H;
      },
      "gn step");

  return true;
}

double LIO::ConstructGICPConstraints(Eigen::Matrix<double, 15, 15>& H,
                                     Eigen::Matrix<double, 15, 1>& b) {
  Eigen::Matrix<double, 8, 6> result_matrix =
      Eigen::Matrix<double, 8, 6>::Zero();
  Eigen::Matrix<double, 8, 6> init_matrix = Eigen::Matrix<double, 8, 6>::Zero();

  if (need_converge_) {
    result_matrix = tbb::parallel_reduce(
        tbb::blocked_range<size_t>(0, correspondences_array_.size()),
        init_matrix,
        [&, this](tbb::blocked_range<size_t> r,
                  Eigen::Matrix<double, 8, 6> local_result) {
          for (size_t i = r.begin(); i < r.end(); ++i) {
            Eigen::Vector3d trans_mean_A =
                curr_state_.pose.block<3, 3>(0, 0) *
                    correspondences_array_[i]->mean_A +
                curr_state_.pose.block<3, 1>(0, 3);

            Eigen::Vector3d error =
                correspondences_array_[i]->mean_B - trans_mean_A;

            // without loss function
            // local_result(7, 0) += gicp_constraint_gain_ * error.transpose() *
            //                       correspondences_array_[i]->mahalanobis *
            //                       error;

            // // The residual takes the partial derivative of the state
            // Eigen::Matrix<double, 3, 6> dres_dx =
            //     Eigen::Matrix<double, 3, 6>::Zero();

            // // The residual takes the partial derivative of rotation
            // dres_dx.block<3, 3>(0, 0) =
            //     curr_state_.pose.block<3, 3>(0, 0) *
            //     Sophus::SO3d::hat(correspondences_array_[i]->mean_A);

            // // The residual takes the partial derivative of position
            // dres_dx.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

            // local_result.block(0, 0, 6, 6) +=
            //     gicp_constraint_gain_ * dres_dx.transpose() *
            //     correspondences_array_[i]->mahalanobis * dres_dx;

            // local_result.block(6, 0, 1, 6) +=
            //     (gicp_constraint_gain_ * dres_dx.transpose() *
            //      correspondences_array_[i]->mahalanobis * error)
            //         .transpose();

            // loss function
            Eigen::Matrix3d mahalanobis =
                correspondences_array_[i]->mahalanobis;
            double cost_function = error.transpose() * mahalanobis * error;
            Eigen::Vector3d rho;
            CauchyLossFunction(cost_function, 10.0, rho);

            local_result(7, 0) += config_.gicp_constraint_gain * rho[0];

            // The residual takes the partial derivative of the state
            Eigen::Matrix<double, 3, 6> dres_dx =
                Eigen::Matrix<double, 3, 6>::Zero();

            // The residual takes the partial derivative of rotation
            dres_dx.block<3, 3>(0, 0) =
                curr_state_.pose.block<3, 3>(0, 0) *
                Sophus::SO3d::hat(correspondences_array_[i]->mean_A);

            // The residual takes the partial derivative of position
            dres_dx.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

            Eigen::Matrix3d robust_information_matrix =
                config_.gicp_constraint_gain *
                (rho[1] * mahalanobis + 2.0 * rho[2] * mahalanobis * error *
                                            error.transpose() * mahalanobis);
            local_result.block(0, 0, 6, 6) +=
                dres_dx.transpose() * robust_information_matrix * dres_dx;

            local_result.block(6, 0, 1, 6) +=
                (config_.gicp_constraint_gain * rho[1] * dres_dx.transpose() *
                 mahalanobis * error)
                    .transpose();
          }

          return local_result;
        },
        [](Eigen::Matrix<double, 8, 6> x, Eigen::Matrix<double, 8, 6> y) {
          return x + y;
        });

    H.block<6, 6>(IndexErrorOri, IndexErrorOri) +=
        result_matrix.block<6, 6>(0, 0);
    b.block<6, 1>(IndexErrorOri, 0) +=
        result_matrix.block<1, 6>(6, 0).transpose();

    return result_matrix(7, 0);
  }

  size_t delta_p_size = voxel_map_ptr_->delta_P_.size();
  size_t N = cloud_cov_ptr_->size();
  correspondences_array_.clear();
  result_matrix = tbb::parallel_reduce(
      tbb::blocked_range<size_t>(0, N),
      init_matrix,
      [&, this](tbb::blocked_range<size_t> r,
                Eigen::Matrix<double, 8, 6> local_result) {
        for (size_t i = r.begin(); i < r.end(); ++i) {
          const PointCovType& point_cov = cloud_cov_ptr_->points[i];
          const Eigen::Vector3d mean_A =
              point_cov.getVector3fMap().cast<double>();
          const Eigen::Vector3d trans_mean_A =
              curr_state_.pose.block<3, 3>(0, 0) * mean_A +
              curr_state_.pose.block<3, 1>(0, 3);

          Eigen::Matrix3d cov_A;
          cov_A << point_cov.cov[0], point_cov.cov[1], point_cov.cov[2],
              point_cov.cov[1], point_cov.cov[3], point_cov.cov[4],
              point_cov.cov[2], point_cov.cov[4], point_cov.cov[5];

          Eigen::Vector3d mean_B = Eigen::Vector3d::Zero();
          Eigen::Matrix3d cov_B = Eigen::Matrix3d::Zero();

          for (size_t i = 0; i < delta_p_size; ++i) {
            Eigen::Vector3d nearby_point =
                trans_mean_A + voxel_map_ptr_->delta_P_[i];
            size_t hash_idx = voxel_map_ptr_->ComputeHashIndex(nearby_point);
            if (voxel_map_ptr_->GetCentroidAndCovariance(
                    hash_idx, mean_B, cov_B) &&
                voxel_map_ptr_->IsSameGrid(nearby_point, mean_B)) {
              Eigen::Matrix3d mahalanobis =
                  (cov_B +
                   curr_state_.pose.block<3, 3>(0, 0) * cov_A *
                       curr_state_.pose.block<3, 3>(0, 0).transpose() +
                   Eigen::Matrix3d::Identity() * 1e-3)
                      .inverse();

              Eigen::Vector3d error = mean_B - trans_mean_A;
              double chi2_error = error.transpose() * mahalanobis * error;
              if (config_.enable_outlier_rejection) {
                if (iter_num_ > 2 && chi2_error > 7.815) {
                  continue;
                }
              }

              std::shared_ptr<Correspondence> corr_ptr =
                  std::make_shared<Correspondence>();
              corr_ptr->mean_A = mean_A;
              corr_ptr->mean_B = mean_B;
              corr_ptr->mahalanobis = mahalanobis;
              correspondences_array_.emplace_back(corr_ptr);

              // without loss function
              // local_result(7, 0) += gicp_constraint_gain_ * chi2_error;

              // // The residual takes the partial derivative of the state
              // Eigen::Matrix<double, 3, 6> dres_dx =
              //     Eigen::Matrix<double, 3, 6>::Zero();

              // // The residual takes the partial derivative of rotation
              // dres_dx.block<3, 3>(0, 0) = curr_state_.pose.block<3, 3>(0, 0)
              // *
              //                             Sophus::SO3d::hat(mean_A);

              // // The residual takes the partial derivative of position
              // dres_dx.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

              // local_result.block(0, 0, 6, 6) += gicp_constraint_gain_ *
              //                                   dres_dx.transpose() *
              //                                   mahalanobis * dres_dx;

              // local_result.block(6, 0, 1, 6) +=
              //     (gicp_constraint_gain_ * dres_dx.transpose() * mahalanobis
              //     *
              //      error)
              //         .transpose();

              // loss function
              double cost_function = chi2_error;
              Eigen::Vector3d rho;
              CauchyLossFunction(cost_function, 10.0, rho);

              local_result(7, 0) += config_.gicp_constraint_gain * rho[0];

              // The residual takes the partial derivative of the state
              Eigen::Matrix<double, 3, 6> dres_dx =
                  Eigen::Matrix<double, 3, 6>::Zero();

              // The residual takes the partial derivative of rotation
              dres_dx.block<3, 3>(0, 0) = curr_state_.pose.block<3, 3>(0, 0) *
                                          Sophus::SO3d::hat(mean_A);

              // The residual takes the partial derivative of position
              dres_dx.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

              Eigen::Matrix3d robust_information_matrix =
                  config_.gicp_constraint_gain *
                  (rho[1] * mahalanobis + 2.0 * rho[2] * mahalanobis * error *
                                              error.transpose() * mahalanobis);
              local_result.block(0, 0, 6, 6) +=
                  dres_dx.transpose() * robust_information_matrix * dres_dx;

              local_result.block(6, 0, 1, 6) +=
                  (config_.gicp_constraint_gain * rho[1] * dres_dx.transpose() *
                   mahalanobis * error)
                      .transpose();

              break;
            }
          }
        }

        return local_result;
      },
      [](Eigen::Matrix<double, 8, 6> x, Eigen::Matrix<double, 8, 6> y) {
        return x + y;
      });

  effect_feat_num_ = correspondences_array_.size();

  H.block<6, 6>(IndexErrorOri, IndexErrorOri) +=
      result_matrix.block<6, 6>(0, 0);
  b.block<6, 1>(IndexErrorOri, 0) +=
      result_matrix.block<1, 6>(6, 0).transpose();

  return result_matrix(7, 0);
}

double LIO::ConstructPoint2PlaneConstraints(Eigen::Matrix<double, 15, 15>& H,
                                            Eigen::Matrix<double, 15, 1>& b) {
  Eigen::Matrix<double, 8, 6> result_matrix =
      Eigen::Matrix<double, 8, 6>::Zero();
  Eigen::Matrix<double, 8, 6> init_matrix = Eigen::Matrix<double, 8, 6>::Zero();

  // Skip the KNN to accelerate convergence
  if (need_converge_) {
    result_matrix = tbb::parallel_reduce(
        tbb::blocked_range<size_t>(0, correspondences_array_.size()),
        init_matrix,
        [&, this](tbb::blocked_range<size_t> r,
                  Eigen::Matrix<double, 8, 6> local_result) {
          for (size_t i = r.begin(); i < r.end(); ++i) {
            const Eigen::Vector3d trans_pt =
                curr_state_.pose.block<3, 3>(0, 0) *
                    correspondences_array_[i]->mean_A +
                curr_state_.pose.block<3, 1>(0, 3);
            const Eigen::Vector4d& plane_coeff =
                correspondences_array_[i]->plane_coeff;

            double error =
                plane_coeff.head(3).dot(trans_pt) + plane_coeff(3, 0);

            local_result(7, 0) +=
                config_.point2plane_constraint_gain * error * error;

            // The residual takes the partial derivative of the state
            Eigen::Matrix<double, 1, 6> dres_dx =
                Eigen::Matrix<double, 1, 6>::Zero();

            // The residual takes the partial derivative of rotation
            dres_dx.block<1, 3>(0, 0) =
                -plane_coeff.head(3).transpose() *
                curr_state_.pose.block<3, 3>(0, 0) *
                Sophus::SO3d::hat(correspondences_array_[i]->mean_A);

            // The residual takes the partial derivative of position
            dres_dx.block<1, 3>(0, 3) = plane_coeff.head(3).transpose();

            local_result.block(0, 0, 6, 6) +=
                config_.point2plane_constraint_gain * dres_dx.transpose() *
                dres_dx;

            local_result.block(6, 0, 1, 6) +=
                config_.point2plane_constraint_gain * dres_dx * error;
          }

          return local_result;
        },
        [](Eigen::Matrix<double, 8, 6> x, Eigen::Matrix<double, 8, 6> y) {
          return x + y;
        });

    H.block<6, 6>(IndexErrorOri, IndexErrorOri) +=
        result_matrix.block<6, 6>(0, 0);
    b.block<6, 1>(IndexErrorOri, 0) +=
        result_matrix.block<1, 6>(6, 0).transpose();

    return result_matrix(7, 0);
  }

  size_t N = cloud_cov_ptr_->size();
  correspondences_array_.clear();
  result_matrix = tbb::parallel_reduce(
      tbb::blocked_range<size_t>(0, N),
      init_matrix,
      [&, this](tbb::blocked_range<size_t> r,
                Eigen::Matrix<double, 8, 6> local_result) {
        for (size_t i = r.begin(); i < r.end(); ++i) {
          const Eigen::Vector3d p =
              cloud_cov_ptr_->points[i].getVector3fMap().cast<double>();
          const Eigen::Vector3d p_w = curr_state_.pose.block<3, 3>(0, 0) * p +
                                      curr_state_.pose.block<3, 1>(0, 3);

          std::vector<Eigen::Vector3d> nearest_points;
          nearest_points.reserve(10);

          voxel_map_ptr_->KNNByCondition(p_w, 5, 5.0, nearest_points);

          Eigen::Vector4d plane_coeff;
          if (nearest_points.size() >= 3 &&
              EstimatePlane(plane_coeff, nearest_points)) {
            double error = plane_coeff.head(3).dot(p_w) + plane_coeff(3, 0);

            bool is_vaild = p.norm() > (81 * error * error);
            if (is_vaild) {
              std::shared_ptr<Correspondence> corr_ptr =
                  std::make_shared<Correspondence>();
              corr_ptr->mean_A = p;
              corr_ptr->plane_coeff = plane_coeff;
              correspondences_array_.emplace_back(corr_ptr);

              local_result(7, 0) +=
                  config_.point2plane_constraint_gain * error * error;

              // The residual takes the partial derivative of the state
              Eigen::Matrix<double, 1, 6> dres_dx =
                  Eigen::Matrix<double, 1, 6>::Zero();

              // The residual takes the partial derivative of rotation
              dres_dx.block<1, 3>(0, 0) = -plane_coeff.head(3).transpose() *
                                          curr_state_.pose.block<3, 3>(0, 0) *
                                          Sophus::SO3d::hat(p);

              // The residual takes the partial derivative of position
              dres_dx.block<1, 3>(0, 3) = plane_coeff.head(3).transpose();

              local_result.block(0, 0, 6, 6) +=
                  config_.point2plane_constraint_gain * dres_dx.transpose() *
                  dres_dx;

              local_result.block(6, 0, 1, 6) +=
                  config_.point2plane_constraint_gain * dres_dx * error;
            }
          }
        }

        return local_result;
      },
      [](Eigen::Matrix<double, 8, 6> x, Eigen::Matrix<double, 8, 6> y) {
        return x + y;
      });

  effect_feat_num_ = correspondences_array_.size();

  H.block<6, 6>(IndexErrorOri, IndexErrorOri) +=
      result_matrix.block<6, 6>(0, 0);
  b.block<6, 1>(IndexErrorOri, 0) +=
      result_matrix.block<1, 6>(6, 0).transpose();

  return result_matrix(7, 0);
}

double LIO::ConstructImuPriorConstraints(Eigen::Matrix<double, 15, 15>& H,
                                         Eigen::Matrix<double, 15, 1>& b) {
  Sophus::SO3d ori_diff =
      Sophus::SO3d(prev_state_.pose.block<3, 3>(0, 0).transpose() *
                   curr_state_.pose.block<3, 3>(0, 0));
  Eigen::Vector3d ori_error = ori_diff.log();

  Eigen::Matrix3d right_jacoiban_inv = Sophus::SO3d::jr_inv(ori_diff);

  Eigen::Matrix<double, 15, 15> jacobian =
      Eigen::Matrix<double, 15, 15>::Identity();
  jacobian.block<3, 3>(IndexErrorOri, IndexErrorOri) = right_jacoiban_inv;

  // LOG(INFO) << "imu jacobian: " << std::endl << jacobian;

  Eigen::Matrix<double, 15, 1> residual = Eigen::Matrix<double, 15, 1>::Zero();
  residual.block<3, 1>(IndexErrorOri, 0) = ori_error;
  residual.block<3, 1>(IndexErrorPos, 0) =
      curr_state_.pose.block<3, 1>(0, 3) - prev_state_.pose.block<3, 1>(0, 3);
  residual.block<3, 1>(IndexErrorVel, 0) = curr_state_.vel - prev_state_.vel;
  residual.block<3, 1>(IndexErrorBiasAcc, 0) = curr_state_.ba - prev_state_.ba;
  residual.block<3, 1>(IndexErrorBiasGyr, 0) = curr_state_.bg - prev_state_.bg;

  Eigen::Matrix<double, 15, 15> inv_P = P_.inverse();

  // LOG(INFO) << "inv_P: " << std::endl << inv_P;

  H += jacobian.transpose() * inv_P * jacobian;
  b += jacobian.transpose() * inv_P * residual;

  double errors = residual.transpose() * inv_P * residual;

  return errors;
}

bool LIO::Predict(const double time,
                  const Eigen::Vector3d& acc_1,
                  const Eigen::Vector3d& gyr_1) {
  double dt = time - lio_time_;

  Eigen::Vector3d un_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d un_gyr = Eigen::Vector3d::Zero();

  NominalStateUpdate(dt,
                     acc_0_,
                     acc_1,
                     gyr_0_,
                     gyr_1,
                     prev_state_.pose,
                     prev_state_.vel,
                     curr_state_.pose,
                     curr_state_.vel,
                     un_acc,
                     un_gyr);

  ErrorStateUpdate(dt, acc_0_, acc_1, gyr_0_, gyr_1);

  if (config_.enable_undistort) {
    // save the predicted pose for scan undistortion
    PoseHistory ph;
    ph.time_ = time;
    ph.T_ = curr_state_.pose;
    ph.un_acc_ = un_acc;
    ph.un_gyr_ = un_gyr;
    ph.vel_ = curr_state_.vel;
    pose_history_.push_back(ph);
  }

  // save the data for next median integral
  prev_state_ = curr_state_;
  acc_0_ = acc_1;
  gyr_0_ = gyr_1;
  lio_time_ = time;

  return true;
}

bool LIO::NominalStateUpdate(const double dt,
                             const Eigen::Vector3d& acc_0,
                             const Eigen::Vector3d& acc_1,
                             const Eigen::Vector3d& gyr_0,
                             const Eigen::Vector3d& gyr_1,
                             const Eigen::Matrix4d& T_prev,
                             const Eigen::Vector3d& vel_prev,
                             Eigen::Matrix4d& T_curr,
                             Eigen::Vector3d& vel_curr,
                             Eigen::Vector3d& un_acc,
                             Eigen::Vector3d& un_gyr) {
  // update ori
  un_gyr = 0.5 * (gyr_0 + gyr_1) - curr_state_.bg;
  T_curr.block<3, 3>(0, 0) =
      T_prev.block<3, 3>(0, 0) * Sophus::SO3d::exp(un_gyr * dt).matrix();

  Eigen::Vector3d un_acc_0 =
      T_prev.block<3, 3>(0, 0) * (acc_0 - curr_state_.ba);
  Eigen::Vector3d un_acc_1 =
      T_curr.block<3, 3>(0, 0) * (acc_1 - curr_state_.ba);
  un_acc = 0.5 * (un_acc_0 + un_acc_1) - g_;

  // update vel
  vel_curr = vel_prev + un_acc * dt;
  // update pos
  T_curr.block<3, 1>(0, 3) =
      T_prev.block<3, 1>(0, 3) + vel_prev * dt + 0.5 * dt * dt * un_acc;

  return true;
}

bool LIO::ErrorStateUpdate(const double dt,
                           const Eigen::Vector3d& acc_0,
                           const Eigen::Vector3d& acc_1,
                           const Eigen::Vector3d& gyr_0,
                           const Eigen::Vector3d& gyr_1) {
  Eigen::Vector3d w = 0.5 * (gyr_0 + gyr_1) - curr_state_.bg;
  Eigen::Vector3d a0 = acc_0 - curr_state_.ba;
  Eigen::Vector3d a1 = acc_1 - curr_state_.ba;

  Eigen::Matrix3d w_x = Sophus::SO3d::hat(w).matrix();
  Eigen::Matrix3d a0_x = Sophus::SO3d::hat(a0).matrix();
  Eigen::Matrix3d a1_x = Sophus::SO3d::hat(a1).matrix();
  Eigen::Matrix3d I_w_x = Sophus::SO3d::exp(-w * dt).matrix();

  F_.setZero();
  // F_.block<3,3>(IndexErrorVel,IndexErrorOri) =
  //     -0.5 * dt * prev_state_.pose.block<3,3>(0,0) * a0_x
  //     -0.5 * dt * curr_state_.pose.block<3,3>(0,0) * a1_x *
  //     (Eigen::Matrix3d::Identity() - w_x * dt);
  F_.block<3, 3>(IndexErrorVel, IndexErrorOri) =
      -0.5 * dt * prev_state_.pose.block<3, 3>(0, 0) * a0_x -
      0.5 * dt * curr_state_.pose.block<3, 3>(0, 0) * a1_x *
          I_w_x;  // More accurate than above
  F_.block<3, 3>(IndexErrorVel, IndexErrorVel) = Eigen::Matrix3d::Identity();
  F_.block<3, 3>(IndexErrorVel, IndexErrorBiasAcc) =
      -0.5 *
      (prev_state_.pose.block<3, 3>(0, 0) +
       curr_state_.pose.block<3, 3>(0, 0)) *
      dt;
  F_.block<3, 3>(IndexErrorVel, IndexErrorBiasGyr) =
      0.5 * curr_state_.pose.block<3, 3>(0, 0) * a1_x * dt * dt;

  F_.block<3, 3>(IndexErrorPos, IndexErrorPos) = Eigen::Matrix3d::Identity();
  F_.block<3, 3>(IndexErrorPos, IndexErrorOri) =
      0.5 * dt * F_.block<3, 3>(IndexErrorVel, IndexErrorOri);
  F_.block<3, 3>(IndexErrorPos, IndexErrorVel) =
      Eigen::Matrix3d::Identity() * dt;
  F_.block<3, 3>(IndexErrorPos, IndexErrorBiasAcc) =
      0.5 * dt * F_.block<3, 3>(IndexErrorVel, IndexErrorBiasAcc);
  F_.block<3, 3>(IndexErrorPos, IndexErrorBiasGyr) =
      0.5 * dt * F_.block<3, 3>(IndexErrorVel, IndexErrorBiasGyr);

  // F_.block<3,3>(IndexErrorOri,IndexErrorOri) = Eigen::Matrix3d::Identity()
  // - w_x * dt;
  F_.block<3, 3>(IndexErrorOri, IndexErrorOri) =
      I_w_x;  // More accurate than above
  F_.block<3, 3>(IndexErrorOri, IndexErrorBiasGyr) =
      -Eigen::Matrix3d::Identity() * dt;

  F_.block<3, 3>(IndexErrorBiasAcc, IndexErrorBiasAcc) =
      Eigen::Matrix3d::Identity();
  F_.block<3, 3>(IndexErrorBiasGyr, IndexErrorBiasGyr) =
      Eigen::Matrix3d::Identity();

  B_.setZero();
  B_.block<3, 3>(IndexErrorVel, IndexNoiseAccLast) =
      0.5 * prev_state_.pose.block<3, 3>(0, 0) * dt;
  B_.block<3, 3>(IndexErrorVel, IndexNoiseGyrLast) =
      -0.25 * curr_state_.pose.block<3, 3>(0, 0) * a1_x * dt * dt;
  B_.block<3, 3>(IndexErrorVel, IndexNoiseAccCurr) =
      0.5 * curr_state_.pose.block<3, 3>(0, 0) * dt;
  B_.block<3, 3>(IndexErrorVel, IndexNoiseGyrCurr) =
      B_.block<3, 3>(IndexErrorVel, IndexNoiseGyrLast);

  B_.block<3, 3>(IndexErrorOri, IndexNoiseGyrLast) =
      0.5 * Eigen::Matrix3d::Identity() * dt;  // inaccuracy
  B_.block<3, 3>(IndexErrorOri, IndexNoiseGyrCurr) =
      B_.block<3, 3>(IndexErrorOri, IndexNoiseGyrLast);

  B_.block<3, 3>(IndexErrorPos, IndexNoiseAccLast) =
      0.5 * B_.block<3, 3>(IndexErrorVel, IndexNoiseAccLast) * dt;
  B_.block<3, 3>(IndexErrorPos, IndexNoiseGyrLast) =
      0.5 * B_.block<3, 3>(IndexErrorVel, IndexNoiseGyrLast) * dt;
  B_.block<3, 3>(IndexErrorPos, IndexNoiseAccCurr) =
      0.5 * B_.block<3, 3>(IndexErrorVel, IndexNoiseAccCurr) * dt;
  B_.block<3, 3>(IndexErrorPos, IndexNoiseGyrCurr) =
      B_.block<3, 3>(IndexErrorPos, IndexNoiseGyrLast);

  B_.block<3, 3>(IndexErrorBiasAcc, IndexNoiseBiasAcc) =
      Eigen::Matrix3d::Identity() * dt;
  B_.block<3, 3>(IndexErrorBiasGyr, IndexNoiseBiasGyr) =
      B_.block<3, 3>(IndexErrorBiasAcc, IndexNoiseBiasAcc);

  P_ = F_ * P_ * F_.transpose() + B_ * Q_ * B_.transpose();
  return true;
}

// Undistortion based on median integral
bool LIO::UndistortPointCloud(const double bag_time,
                              const double lidar_end_time,
                              CloudPtr& cloud_ptr) {
  Eigen::Matrix3d R_w_be = curr_state_.pose.block<3, 3>(0, 0);
  Eigen::Vector3d t_w_be = curr_state_.pose.block<3, 1>(0, 3);
  auto it_pt = cloud_ptr->points.end() - 1;
  bool finshed_flag = false;
  for (auto it_pose = pose_history_.end() - 1; it_pose != pose_history_.begin();
       --it_pose) {
    auto bi = it_pose - 1;
    auto bj = it_pose;
    Eigen::Matrix3d R_w_bi = bi->T_.block<3, 3>(0, 0);
    Eigen::Vector3d t_w_bi = bi->T_.block<3, 1>(0, 3);
    Eigen::Vector3d v_w_bi = bi->vel_;
    Eigen::Vector3d un_acc_bj = bj->un_acc_;
    Eigen::Vector3d un_gyr_bj = bj->un_gyr_;

    for (; (it_pt->curvature / (double)(1000) + bag_time) > bi->time_;
         --it_pt) {
      double dt = (it_pt->curvature / (double)(1000) + bag_time) - bi->time_;

      Eigen::Matrix3d R_w_bk =
          R_w_bi * Sophus::SO3d::exp(un_gyr_bj * dt).matrix();
      Eigen::Vector3d t_w_bebk =
          t_w_bi + v_w_bi * dt + 0.5 * dt * dt * un_acc_bj - t_w_be;
      // point_K
      Eigen::Vector3d P_bk_bkK(it_pt->x, it_pt->y, it_pt->z);
      Eigen::Vector3d P_w_beK =
          R_w_be.transpose() * (R_w_bk * P_bk_bkK + t_w_bebk);

      it_pt->x = P_w_beK.x();
      it_pt->y = P_w_beK.y();
      it_pt->z = P_w_beK.z();

      if (it_pt == cloud_ptr->points.begin()) {
        finshed_flag = true;
        break;
      }
    }

    if (finshed_flag) {
      break;
    }
  }

  // Remove excess history imu_pose
  while (!pose_history_.empty() &&
         (pose_history_.front().time_ < lidar_end_time)) {
    pose_history_.pop_front();
  }

  return true;
}

bool LIO::StaticInitialization(SensorMeasurement& sensor_measurement) {
  if (first_imu_frame_) {
    const auto& acc = sensor_measurement.imu_buff_.front().linear_acceleration;
    const auto& gyr = sensor_measurement.imu_buff_.front().angular_velocity;
    imu_init_buff_.emplace_back(Eigen::Vector3d(acc.x, acc.y, acc.z),
                                Eigen::Vector3d(gyr.x, gyr.y, gyr.z));
  }

  for (const auto& imu_msg : sensor_measurement.imu_buff_) {
    Eigen::Vector3d acc(imu_msg.linear_acceleration.x,
                        imu_msg.linear_acceleration.y,
                        imu_msg.linear_acceleration.z);
    Eigen::Vector3d gyr(imu_msg.angular_velocity.x,
                        imu_msg.angular_velocity.y,
                        imu_msg.angular_velocity.z);

    imu_init_buff_.emplace_back(acc, gyr);
  }

  if (imu_init_buff_.size() < max_init_count_) {
    return false;
  }

  Eigen::Vector3d acc_cov, gyr_cov;
  ComputeMeanAndCovDiag(
      imu_init_buff_,
      mean_acc_,
      acc_cov,
      [](const std::pair<Eigen::Vector3d, Eigen::Vector3d>& imu_data) {
        return imu_data.first;
      });
  ComputeMeanAndCovDiag(
      imu_init_buff_,
      mean_gyr_,
      gyr_cov,
      [](const std::pair<Eigen::Vector3d, Eigen::Vector3d>& imu_data) {
        return imu_data.second;
      });

  // Compute initial attitude via Schmidt orthogonalization.
  // The roll and pitch are aligned with the direction of gravity, but the yaw
  // is random.
  Eigen::Vector3d z_axis = mean_acc_.normalized();
  Eigen::Vector3d e1(1, 0, 0);
  Eigen::Vector3d x_axis = e1 - z_axis * z_axis.transpose() * e1;
  x_axis.normalize();
  Eigen::Vector3d y_axis = Sophus::SO3d::hat(z_axis).matrix() * x_axis;
  y_axis.normalize();

  Eigen::Matrix3d init_R;
  init_R.block<3, 1>(0, 0) = x_axis;
  init_R.block<3, 1>(0, 1) = y_axis;
  init_R.block<3, 1>(0, 2) = z_axis;
  Eigen::Quaterniond init_q(init_R);
  curr_state_.pose.block<3, 3>(0, 0) =
      init_q.normalized().toRotationMatrix().transpose();

  Eigen::Vector3d init_ba = Eigen::Vector3d::Zero();
  ComputeMeanAndCovDiag(
      imu_init_buff_,
      init_ba,
      acc_cov,
      [this](const std::pair<Eigen::Vector3d, Eigen::Vector3d>& imu_data) {
        Eigen::Vector3d temp_ba =
            imu_data.first -
            curr_state_.pose.block<3, 3>(0, 0).transpose() * g_;
        return temp_ba;
      });

  // init pose
  curr_state_.pose.block<3, 1>(0, 3).setZero();
  // init velocity
  curr_state_.vel.setZero();
  // init bg
  curr_state_.bg = mean_gyr_;
  // init ba
  curr_state_.ba = init_ba;

  prev_state_ = curr_state_;

  P_.setIdentity();
  P_.block<3, 3>(IndexErrorOri, IndexErrorOri) =
      config_.init_ori_cov * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(IndexErrorPos, IndexErrorPos) =
      config_.init_pos_cov * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(IndexErrorVel, IndexErrorVel) =
      config_.init_vel_cov * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(IndexErrorBiasAcc, IndexErrorBiasAcc) =
      config_.init_ba_cov * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(IndexErrorBiasGyr, IndexErrorBiasGyr) =
      config_.init_bg_cov * Eigen::Matrix3d::Identity();

  Q_.setIdentity();
  Q_.block<3, 3>(IndexNoiseAccLast, IndexNoiseAccLast) =
      config_.acc_cov * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(IndexNoiseGyrLast, IndexNoiseGyrLast) =
      config_.gyr_cov * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(IndexNoiseAccCurr, IndexNoiseAccCurr) =
      config_.acc_cov * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(IndexNoiseGyrCurr, IndexNoiseGyrCurr) =
      config_.gyr_cov * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(IndexNoiseBiasAcc, IndexNoiseBiasAcc) =
      config_.ba_cov * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(IndexNoiseBiasGyr, IndexNoiseBiasGyr) =
      config_.bg_cov * Eigen::Matrix3d::Identity();

  lio_time_ = sensor_measurement.imu_buff_.back().header.stamp.toSec();
  lio_init_ = true;

  LOG(INFO) << "imu static, mean_acc_: " << mean_acc_.transpose()
            << " init_ba: " << init_ba.transpose() << ", ori: " << std::endl
            << curr_state_.pose.block<3, 3>(0, 0);

  return true;
}

bool LIO::AHRSInitialization(SensorMeasurement& sensor_measurement) {
  const auto& back_imu = sensor_measurement.imu_buff_.back();

  if ((back_imu.orientation.w * back_imu.orientation.w +
       back_imu.orientation.x * back_imu.orientation.x +
       back_imu.orientation.y * back_imu.orientation.y +
       back_imu.orientation.z * back_imu.orientation.z) < 1.0) {
    LOG(ERROR) << "AHRS initalization falid, please use static initalizaiton!";
    return false;
  }

  Eigen::Quaterniond temp_q(back_imu.orientation.w,
                            back_imu.orientation.x,
                            back_imu.orientation.y,
                            back_imu.orientation.z);

  curr_state_.pose.block<3, 3>(0, 0) = temp_q.toRotationMatrix();

  curr_state_.pose.block<3, 1>(0, 3).setZero();

  curr_state_.vel.setZero();

  curr_state_.bg.setZero();

  curr_state_.ba.setZero();

  prev_state_ = curr_state_;

  P_.setIdentity();
  P_.block<3, 3>(IndexErrorOri, IndexErrorOri) =
      config_.init_ori_cov * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(IndexErrorPos, IndexErrorPos) =
      config_.init_pos_cov * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(IndexErrorVel, IndexErrorVel) =
      config_.init_vel_cov * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(IndexErrorBiasAcc, IndexErrorBiasAcc) =
      config_.init_ba_cov * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(IndexErrorBiasGyr, IndexErrorBiasGyr) =
      config_.init_bg_cov * Eigen::Matrix3d::Identity();

  Q_.setIdentity();
  Q_.block<3, 3>(IndexNoiseAccLast, IndexNoiseAccLast) =
      config_.acc_cov * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(IndexNoiseGyrLast, IndexNoiseGyrLast) =
      config_.gyr_cov * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(IndexNoiseAccCurr, IndexNoiseAccCurr) =
      config_.acc_cov * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(IndexNoiseGyrCurr, IndexNoiseGyrCurr) =
      config_.gyr_cov * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(IndexNoiseBiasAcc, IndexNoiseBiasAcc) =
      config_.ba_cov * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(IndexNoiseBiasGyr, IndexNoiseBiasGyr) =
      config_.bg_cov * Eigen::Matrix3d::Identity();

  lio_time_ = sensor_measurement.imu_buff_.back().header.stamp.toSec();
  lio_init_ = true;

  return true;
}

bool LIO::CorrectState(const State& state,
                       const Eigen::Matrix<double, 15, 1>& delta_x,
                       State& corrected_state) {
  // ori
  Eigen::Matrix3d delta_R =
      Sophus::SO3d::exp(delta_x.block<3, 1>(IndexErrorOri, 0)).matrix();
  // The normalization is employed after each update to pervent numerical
  // stability
  Eigen::Quaterniond temp_q(state.pose.block<3, 3>(0, 0) * delta_R);
  temp_q.normalize();
  corrected_state.pose.block<3, 3>(0, 0) = temp_q.toRotationMatrix();
  // pos
  corrected_state.pose.block<3, 1>(0, 3) =
      state.pose.block<3, 1>(0, 3) + delta_x.block<3, 1>(IndexErrorPos, 0);
  // vel
  corrected_state.vel = state.vel + delta_x.block<3, 1>(IndexErrorVel, 0);
  // ba
  corrected_state.ba = state.ba + delta_x.block<3, 1>(IndexErrorBiasAcc, 0);
  // bg
  corrected_state.bg = state.bg + delta_x.block<3, 1>(IndexErrorBiasGyr, 0);

  return true;
}

bool LIO::ComputeFinalCovariance(const Eigen::Matrix<double, 15, 1>& delta_x) {
  Eigen::Matrix<double, 15, 15> temp_P = final_hessian_.inverse();

  // project covariance
  Eigen::Matrix<double, 15, 15> L = Eigen::Matrix<double, 15, 15>::Identity();
  L.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() -
                        0.5 * Sophus::SO3d::hat(delta_x.block<3, 1>(0, 0));
  P_ = L * temp_P * L;

  return true;
}

bool LIO::IsConverged(const Eigen::Matrix<double, 15, 1>& delta_x) {
  return delta_x.lpNorm<Eigen::Infinity>() < transformation_epsilon_;
}
