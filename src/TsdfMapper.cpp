/**
 * Copyright (C) 2023-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author Kin ZHANG (https://kin-zhang.github.io/)
 * @date: 2023-05-02 21:23
 * @details: No ROS version, speed up the process, check our benchmark in dufomap
 */
#include "common/TsdfMapper.h"
namespace voxblox
{
    TsdfMapper::TsdfMapper(common::Config& config){
        TsdfMap::Config TsdfMap_config;
        // TsdfMap_config.tsdf_voxel_size = config.tsdf_voxel_size_;
        // TsdfMap_config.tsdf_voxels_per_side = config.tsdf_voxels_per_side_;

        // Initialize TSDF Map and integrator.
        tsdf_map_.reset(new TsdfMap(TsdfMap_config));
        LOG(INFO) << tsdf_map_->block_size();
    }

    void TsdfMapper::processPointCloudMessageAndInsert(pcl::PointCloud<PointType>::Ptr const& single_pc){
        // integratePointcloud(T_G_C_refined, points_C, colors, is_freespace_pointcloud);
        if (config_.verbose_) {
            LOG(INFO) << "have " <<  tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() << " blocks.";
        }

        // tsdf_map_->getTsdfLayerPtr()->removeDistantBlocks(
        //     T_G_C.getPosition(), max_block_distance_from_body_);
        // mesh_layer_->clearDistantMesh(T_G_C.getPosition(),
        //                                 max_block_distance_from_body_);

        // Callback for inheriting classes.
        // newPoseCallback(T_G_C);
    }
}