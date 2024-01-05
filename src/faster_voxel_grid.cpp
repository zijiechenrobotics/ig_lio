#include "ig_lio/faster_voxel_grid.h"

size_t FasterVoxelGrid::ComputeHashIndex(const Eigen::Vector3d& point) {
  double loc_xyz[3];
  for (size_t i = 0; i < 3; ++i) {
    loc_xyz[i] = point[i] * inv_resolution_;
    if (loc_xyz[i] < 0) {
      loc_xyz[i] -= 1.0;
    }
  }

  size_t x = static_cast<size_t>(loc_xyz[0]);
  size_t y = static_cast<size_t>(loc_xyz[1]);
  size_t z = static_cast<size_t>(loc_xyz[2]);

  return ((((z)*HASH_P_) % MAX_N_ + (y)) * HASH_P_) % MAX_N_ + (x);
}

void FasterVoxelGrid::Filter(const CloudPtr& input_cloud_ptr,
                             CloudPtr& cloud_DS_ptr,
                             CloudCovPtr& cloud_cov_ptr) {
  voxel_array_ptr_->reserve(input_cloud_ptr->size());

  // Assign each points to the voxel_map
  tbb::parallel_for(tbb::blocked_range<size_t>(0, input_cloud_ptr->size()),
                    [&, this](tbb::blocked_range<size_t> r) {
                      for (size_t i = r.begin(); i < r.end(); ++i) {
                        const Eigen::Vector3d point =
                            input_cloud_ptr->points[i]
                                .getVector3fMap()
                                .template cast<double>();
                        size_t hash_idx = ComputeHashIndex(point);

                        MyAccessor accessor;
                        voxel_map_ptr_->insert(accessor, hash_idx);

                        // The grid does not exist, create it
                        if (accessor->second == nullptr) {
                          accessor->second = std::make_shared<Voxel>();

                          accessor->second->centorid_ = point;
                          accessor->second->N_++;

                          voxel_array_ptr_->emplace_back(accessor->second);
                        }
                        // The grid already exists, update it
                        else {
                          accessor->second->N_++;
                          accessor->second->centorid_ +=
                              (point - accessor->second->centorid_) /
                              static_cast<double>(accessor->second->N_);
                        }
                      }
                    });

  // Voxel-based surface covariance estimator
  cloud_DS_ptr->resize(voxel_array_ptr_->size());
  cloud_cov_ptr->resize(voxel_array_ptr_->size());
  size_t point_with_cov_count = 0;
  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, voxel_array_ptr_->size()),
      [&, this](tbb::blocked_range<size_t> r) {
        for (size_t i = r.begin(); i < r.end(); ++i) {
          cloud_DS_ptr->points[i].getVector3fMap() =
              voxel_array_ptr_->at(i)->centorid_.cast<float>();

          cloud_cov_ptr->points[i].getVector3fMap() =
              voxel_array_ptr_->at(i)->centorid_.cast<float>();

          Eigen::Matrix3d modified_cov = Eigen::Matrix3d::Zero();
          size_t points_num = 0;
          Eigen::Vector3d points_sum = Eigen::Vector3d::Zero();
          Eigen::Matrix3d cov_sum = Eigen::Matrix3d::Zero();
          for (size_t j = 0; j < search_range_.size(); ++j) {
            Eigen::Vector3d near_point =
                voxel_array_ptr_->at(i)->centorid_ + search_range_[j];
            size_t hash_idx = ComputeHashIndex(near_point);

            MyAccessor accessor;
            if (voxel_map_ptr_->find(accessor, hash_idx)) {
              points_sum += accessor->second->centorid_;
              cov_sum += accessor->second->centorid_ *
                         accessor->second->centorid_.transpose();
              points_num++;
            }
          }

          if (points_num >= min_points_per_grid_) {
            Eigen::Vector3d centroid =
                points_sum / static_cast<double>(points_num);
            Eigen::Matrix3d cov =
                (cov_sum - points_sum * centroid.transpose()) /
                (static_cast<double>(points_num) - 1.0);

            Eigen::JacobiSVD<Eigen::Matrix3d> svd(
                cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

            Eigen::Vector3d values(1, 1, 1e-3);

            modified_cov =
                svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();

            point_with_cov_count++;
          }

          cloud_cov_ptr->points[i].cov[0] = modified_cov(0, 0);
          cloud_cov_ptr->points[i].cov[1] = modified_cov(0, 1);
          cloud_cov_ptr->points[i].cov[2] = modified_cov(0, 2);
          cloud_cov_ptr->points[i].cov[3] = modified_cov(1, 1);
          cloud_cov_ptr->points[i].cov[4] = modified_cov(1, 2);
          cloud_cov_ptr->points[i].cov[5] = modified_cov(2, 2);
        }
      });

  frame_count_++;
  double current_precent = (static_cast<double>(point_with_cov_count) /
                            static_cast<double>(cloud_cov_ptr->size())) *
                           100.0;
  ava_precent_ +=
      (current_precent - ava_precent_) / static_cast<double>(frame_count_);

  LOG(INFO) << "point with cov size: " << point_with_cov_count
            << " total points: " << cloud_cov_ptr->size()
            << " precent: " << current_precent
            << ", ava_precent: " << ava_precent_;

  // reset voxel_map and voxel_array
  voxel_array_ptr_->clear();
  voxel_map_ptr_->clear();
}
