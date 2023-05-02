/**
 * Copyright (C) 2023-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author Kin ZHANG (https://kin-zhang.github.io/)
 * @date: 2023-05-02 19:43
 * @details: No ROS version, speed up the process, check our benchmark in dufomap
 */

#include "dynablox/clustering.h"
#include "common/neighborhood_search.h"

namespace dynablox
{
    Clustering::Clustering(const common::Config& config, TsdfLayer::Ptr tsdf_layer)
    : config_(config),
      tsdf_layer_(std::move(tsdf_layer)),
      neighborhood_search_(config.neighbor_connectivity) {}

    Clusters Clustering::performClustering(const BlockToPointMap& point_map,
      const ClusterIndices& occupied_ever_free_voxel_indices,
      const int frame_counter, const pcl::PointCloud<PointType>& cloud, CloudInfo& cloud_info) const{
    // Cluster all occupied voxels.
    const std::vector<ClusterIndices> voxel_cluster_indices =
        voxelClustering(occupied_ever_free_voxel_indices, frame_counter);

    // Group points into clusters.
    // Clusters clusters = inducePointClusters(point_map, voxel_cluster_indices);
    // for (Cluster& cluster : clusters) {
    //     computeAABB(cloud, cluster);
    // }

    // // Merge close Clusters.
    // mergeClusters(cloud, clusters);

    // // Apply filters to remove spurious clusters.
    // applyClusterLevelFilters(clusters);

    // // Label all remaining points as dynamic.
    // setClusterLevelDynamicFlagOfallPoints(clusters, cloud_info);
    // return clusters;

    }
    std::vector<Clustering::ClusterIndices> Clustering::voxelClustering(
        const ClusterIndices& occupied_ever_free_voxel_indices,
        const int frame_counter) const {
        std::vector<ClusterIndices> voxel_cluster_indices;

        // Process all newly occupied ever-free voxels as potential cluster seeds.
        for (const voxblox::VoxelKey& voxel_key : occupied_ever_free_voxel_indices) {
            ClusterIndices cluster;
            if (growCluster(voxel_key, frame_counter, cluster)) {
            voxel_cluster_indices.push_back(cluster);
            }
        }
        return voxel_cluster_indices;
    }

    bool Clustering::growCluster(const voxblox::VoxelKey& seed,
                                const int frame_counter,
                                ClusterIndices& result) const {
        std::vector<voxblox::VoxelKey> stack = {seed};
        const size_t voxels_per_side = tsdf_layer_->voxels_per_side();

        while (!stack.empty()) {
            // Get the voxel.
            const voxblox::VoxelKey voxel_key = stack.back();
            stack.pop_back();
            TsdfBlock::Ptr tsdf_block =
                tsdf_layer_->getBlockPtrByIndex(voxel_key.first);
            if (!tsdf_block) {
            continue;
            }
            TsdfVoxel& tsdf_voxel = tsdf_block->getVoxelByVoxelIndex(voxel_key.second);

            // Process every voxel only once.
            if (tsdf_voxel.clustering_processed) {
            continue;
            }

            // Add voxel to cluster.
            tsdf_voxel.dynamic = true;
            tsdf_voxel.clustering_processed = true;
            result.push_back(voxel_key);

            // Extend cluster to neighbor voxels.
            const voxblox::AlignedVector<voxblox::VoxelKey> neighbors =
                neighborhood_search_.search(voxel_key.first, voxel_key.second,
                                            voxels_per_side);

            for (const voxblox::VoxelKey& neighbor_key : neighbors) {
            TsdfBlock::Ptr neighbor_block =
                tsdf_layer_->getBlockPtrByIndex(neighbor_key.first);
            if (!neighbor_block) {
                continue;
            }
            TsdfVoxel& neighbor_voxel =
                neighbor_block->getVoxelByVoxelIndex(neighbor_key.second);

            // If neighbor is valid add it to the cluster, and potentially keep
            // growing if it is ever-free.
            if (!neighbor_voxel.clustering_processed &&
                neighbor_voxel.last_lidar_occupied == frame_counter) {
                if (neighbor_voxel.ever_free ||
                    (tsdf_voxel.ever_free && config_.grow_clusters_twice)) {
                stack.push_back(neighbor_key);
                } else {
                // Add voxel to cluster.
                neighbor_voxel.dynamic = true;
                neighbor_voxel.clustering_processed = true;
                result.push_back(neighbor_key);
                }
            }
            }
        }
        return !result.empty();
    }
} // namespace dynablox