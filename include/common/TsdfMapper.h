/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author Kin ZHANG (https://kin-zhang.github.io/)
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
#include <voxblox/core/voxel.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/projective_tsdf_integrator.h>

#include "common/utils.h"
#include "common/timing.hpp"

namespace voxblox{
class TsdfMapper{
public:
    TsdfMapper(common::Config& config);
    virtual ~TsdfMapper() = default;
    void processPointCloudMessageAndInsert(pcl::PointCloud<PointType>::Ptr const& single_pc);

    std::shared_ptr<TsdfMap> getTsdfMapPtr() { return tsdf_map_; }
    std::shared_ptr<const TsdfMap> getTsdfMapPtr() const { return tsdf_map_; }

private:
    common::Config config_;

protected:
    FloatingPoint block_size_;
    std::shared_ptr<TsdfMap> tsdf_map_;
    TsdfIntegratorBase::Ptr tsdf_integrator_;
};
}