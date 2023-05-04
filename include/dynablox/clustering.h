/**
 * Copyright (C) 2023-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author Kin ZHANG (https://kin-zhang.github.io/)
 * @date: 2023-05-02 19:39
 * @details: No ROS version, speed up the process, check our benchmark in dufomap
 */
#pragma once

#include <memory>
#include <vector>

#include <voxblox/utils/neighbor_tools.h>

#include "dynablox/types.h"

#include "common/neighborhood_search.h"
#include "common/utils.h"

namespace dynablox {
class Clustering {
 private:
  common::Config::ClusterCfg config_;
  const TsdfLayer::Ptr tsdf_layer_;
  const NeighborhoodSearch neighborhood_search_;

 public:
  Clustering(const common::Config::ClusterCfg& config, TsdfLayer::Ptr tsdf_layer);
  virtual ~Clustering() = default;

  /**
   * @brief Cluster all currently occupied voxels that are next to an ever-free
   * voxel. Merge nearby clusters and apply cluster level filters.
   *
   * @param point_map Map of points to voxels.
   * @param occupied_ever_free_voxel_indices Occupied voxels to seed cluster
   * growing.
   * @param frame_counter Current frame number.
   * @param cloud_info Info to store which points are cluster-level dynamic.
   * @return The identified clusters.
   */

  using ClusterIndices = std::vector<voxblox::VoxelKey>;
  Clusters performClustering(const BlockToPointMap& point_map, const ClusterIndices& occupied_ever_free_voxel_indices,
                             const int frame_counter, const Cloud& cloud, CloudInfo& cloud_info) const;

  /**
   * @brief Use the voxel level clustering to assign all points to clusters.
   *
   * @param point_map Mapping of blocks to voxels and points in the cloud.
   * @param voxel_cluster_indices Voxel indices per cluster.
   * @return All clusters.
   */
  Clusters inducePointClusters(const BlockToPointMap& point_map,
                               const std::vector<ClusterIndices>& voxel_cluster_indices) const;

  /**
   * @brief Cluster all currently occupied voxels that are next to an ever-free
   * voxel.
   *
   * @param occupied_ever_free_voxel_indices Occupied ever-free voxel indices to
   * seed the clusters.
   * @param frame_counter Frame number to verify added voxels contain points
   * this scan.
   * @return Vector of all found clusters.
   */
  std::vector<ClusterIndices> voxelClustering(const ClusterIndices& occupied_ever_free_voxel_indices,
                                              const int frame_counter) const;

  /**
   * @brief Grow a single cluster from a seed voxel key. All voxels that are not
   * yet processed are added to the cluster and labeled as processed and
   * dynamic. Only ever-free voxels can further grow the cluster.
   *
   * @param seed Voxel key to start clustering from.
   * @param frame_counter Frame number to verify added voxels contain points
   * this scan.
   * @param result Where to store the voxel keys of all voxels of the cluster.
   * @return If a result cluster was found.
   */
  bool growCluster(const voxblox::VoxelKey& seed, const int frame_counter, ClusterIndices& result) const;

  /**
   * @brief Compute the axis-aligned bounding box for a cluster.
   *
   * @param cloud Pointcloud to look up the positions.
   * @param cluster Clsuter to evaluate.
   */
  void computeAABB(const Cloud& cloud, Cluster& cluster) const;

  /**
   * @brief Merge clusters together whose points are clsoe together.
   *
   * @param cloud Pointcloud to lookup cluster points poses.
   * @param clusters Clsuters to be checked and merged.
   */
  void mergeClusters(const Cloud& cloud, Clusters& clusters) const;

  /**
   * @brief Removes all clusters that don't meet the filtering criteria.
   *
   * @param candidates list of clusters that will be filtered.
   */
  void applyClusterLevelFilters(Clusters& candidates) const;

  /**
   * @brief Check filters for an inidividual cluster.
   *
   * @param cluster Cluster to check.
   * @return True if the cluster was filtered out.
   */
  bool filterCluster(const Cluster& cluster) const;

  /**
   * @brief Sets dynamic flag on point level (includes points belonging to
   * extension of high confidence detection clusters)
   *
   * @param clusters Clusters whose points will be labeled.
   * @param cloud_info Cloud info where the label is placed.
   */
  void setClusterLevelDynamicFlagOfallPoints(const Clusters& clusters, CloudInfo& cloud_info) const;
};

}  // namespace dynablox
