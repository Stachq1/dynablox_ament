/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author Qingwen Zhang (https://kin-zhang.github.io/)
 * @date: 2023-05-02 17:03
 * @details: No ROS version, speed up the process
 *
 * Input: PCD files + Prior raw global map , check our benchmark in dufomap
 * Output: Cleaned global map / Detection
 */
#pragma once

#include <voxblox/core/block.h>
#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/voxel.h>
#include <voxblox/integrator/projective_tsdf_integrator.h>
#include <voxblox/utils/color_maps.h>

#include "dynablox/index_getter.h"
#include "dynablox/types.h"

#include "common/neighborhood_search.h"
#include "common/timing.hpp"
#include "common/utils.h"
namespace voxblox {

constexpr float kDefaultMaxIntensity = 100.0;
class TsdfMapper {
 public:
  TsdfMapper(common::Config::VoxbloxCfg& config);
  virtual ~TsdfMapper() = default;
  void processPointCloudAndInsert(dynablox::Cloud& cloud, voxblox::Transformation& T_G_C, ufo::Timing& timing_);

  std::shared_ptr<TsdfMap> getTsdfMapPtr() { return tsdf_map_; }
  std::shared_ptr<const TsdfMap> getTsdfMapPtr() const { return tsdf_map_; }

 private:
  common::Config::VoxbloxCfg config_;
  std::shared_ptr<ColorMap> color_map_;
  double max_block_distance_from_body_ = std::numeric_limits<FloatingPoint>::max();
  void setColor();

 protected:
  FloatingPoint block_size_;
  std::shared_ptr<TsdfMap> tsdf_map_;
  TsdfIntegratorBase::Ptr tsdf_integrator_;
};
}  // namespace voxblox

namespace dynablox {
class EverFreeIntegrator {
 public:
  EverFreeIntegrator(const common::Config::EverFreeCfg& config, std::shared_ptr<TsdfLayer> tsdf_layer);
  virtual ~EverFreeIntegrator() = default;

  /**
   * @brief Update the ever-free state of all changed TSDF-voxels by checking
   * when they were last occupied and for how long.
   *
   * @param frame_counter Index of current lidar scan to compute age.
   */
  void updateEverFreeVoxels(const int frame_counter, ufo::Timing& timing_) const;

  /**
   * @brief Process each block in parallel.
   *
   * @param block_index Index of block to process.
   * @param frame_counter Index of current lidar scan to compute age.
   * @return All voxels that fell outside the block and need clearing later.
   */
  bool blockWiseUpdateEverFree(const BlockIndex& block_index, const int frame_counter,
                               voxblox::AlignedVector<voxblox::VoxelKey>& voxels_to_remove) const;

  /**
   * @brief If the voxel is currently static we leave it. If it was last static
   * last frame, increment the occupancy counter, else reset it.
   *
   * @param tsdf_voxel Voxel to update.
   * @param frame_counter Current lidar scan time index.
   */
  void updateOccupancyCounter(TsdfVoxel& tsdf_voxel, const int frame_counter) const;

  /**
   * @brief Remove the ever-free and dynamic attributes from a given voxel and
   * all its neighbors (which now also don't meet the criteria anymore.)
   *
   * @param block Tsdf block containing the voxel.
   * @param voxel Voxel to be cleared from ever-free.
   * @param block_index Index of the containing block.
   * @param voxel_index Index of the voxel in the block.
   * @return All voxels that fell outside the block and need clearing later.
   */
  voxblox::AlignedVector<voxblox::VoxelKey> removeEverFree(TsdfBlock& block, TsdfVoxel& voxel,
                                                           const BlockIndex& block_index,
                                                           const VoxelIndex& voxel_index) const;

  /**
   * @brief Check for any occupied or unknown voxels in neighborhood, otherwise
   * mark voxel as ever free. Check all voxels in the block.
   *
   * @param block_index Index of block to check.
   * @param frame_counter Current frame to compute occupied time.
   */
  void blockWiseMakeEverFree(const BlockIndex& block_index, const int frame_counter) const;

 private:
  const common::Config::EverFreeCfg config_;
  const TsdfLayer::Ptr tsdf_layer_;
  const NeighborhoodSearch neighborhood_search_;

  // Cached frequently used values.
  const float voxel_size_;
  const size_t voxels_per_side_;
  const size_t voxels_per_block_;
};

}  // namespace dynablox

namespace voxblox {
// Check if all coordinates in the PCL point are finite.
template <typename PCLPoint>
inline bool isPointFinite(const PCLPoint& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}
template <typename PCLPoint>
Color convertColor(const PCLPoint& point, const std::shared_ptr<ColorMap>& color_map);

template <>
inline Color convertColor(const pcl::PointXYZRGB& point, const std::shared_ptr<ColorMap>& /*color_map*/) {
  return Color(point.r, point.g, point.b, point.a);
}

template <>
inline Color convertColor(const pcl::PointXYZI& point, const std::shared_ptr<ColorMap>& color_map) {
  return color_map->colorLookup(point.intensity);
}

template <>
inline Color convertColor(const pcl::PointXYZ& /*point*/, const std::shared_ptr<ColorMap>& color_map) {
  return color_map->colorLookup(0);
}

/// Convert pointclouds of different PCL types to a voxblox pointcloud.
template <typename PCLPoint>
inline void convertPointcloud(const typename pcl::PointCloud<PCLPoint>& pointcloud_pcl,
                              const std::shared_ptr<ColorMap>& color_map, Pointcloud* points_C, Colors* colors) {
  points_C->reserve(pointcloud_pcl.size());
  colors->reserve(pointcloud_pcl.size());
  for (size_t i = 0; i < pointcloud_pcl.points.size(); ++i) {
    if (!isPointFinite(pointcloud_pcl.points[i])) {
      continue;
    }
    points_C->push_back(Point(pointcloud_pcl.points[i].x, pointcloud_pcl.points[i].y, pointcloud_pcl.points[i].z));
    colors->emplace_back(convertColor<PCLPoint>(pointcloud_pcl.points[i], color_map));
  }
}
}  // namespace voxblox
