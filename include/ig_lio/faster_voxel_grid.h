/*
 * @Description: voxel-based surface covariance estimator
 * @Autor: Zijie Chen
 * @Date: 2023-12-26 20:05:38
 */

#ifndef FASTER_VOXEL_GRID_H_
#define FASTER_VOXEL_GRID_H_

#include <glog/logging.h>

#include <tbb/concurrent_hash_map.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>

#include "point_type.h"

class FasterVoxelGrid {
 public:
  FasterVoxelGrid(double resolution)
      : resolution_(resolution)
      , inv_resolution_(1.0 / resolution) {
    voxel_map_ptr_ = std::make_shared<MyHashMap>();
    voxel_array_ptr_ = std::make_shared<MyVector>();

    search_range_.clear();
    for (int x_gain = -1; x_gain <= 1; ++x_gain) {
      for (int y_gain = -1; y_gain <= 1; ++y_gain) {
        for (int z_gain = -1; z_gain <= 1; ++z_gain) {
          search_range_.emplace_back(Eigen::Vector3d(x_gain * resolution_,
                                                     y_gain * resolution_,
                                                     z_gain * resolution_));
        }
      }
    }
  };

  void Filter(const CloudPtr& input_cloud_ptr,
              CloudPtr& cloud_DS_ptr,
              CloudCovPtr& cloud_cov_ptr);

  size_t ComputeHashIndex(const Eigen::Vector3d& point);

  double resolution_{1.0};
  double inv_resolution_{1.0};

  struct Voxel {
    Voxel(){};

    Eigen::Vector3d centorid_ = Eigen::Vector3d::Zero();
    size_t N_{0};
  };

  using MyHashMap = tbb::concurrent_hash_map<size_t, std::shared_ptr<Voxel>>;
  using MyVector = tbb::concurrent_vector<std::shared_ptr<Voxel>>;
  using MyAccessor = MyHashMap::accessor;

  std::shared_ptr<MyHashMap> voxel_map_ptr_;
  std::shared_ptr<MyVector> voxel_array_ptr_;

  std::vector<Eigen::Vector3d> search_range_;

  double ava_precent_{0.0};
  size_t frame_count_{0};

  size_t HASH_P_{116101};
  size_t MAX_N_{10000000000};
  size_t min_points_per_grid_{6};
};

#endif