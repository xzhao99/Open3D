// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tuple>
#include <vector>

#include "open3d/pipelines/registration/TransformationEstimation.h"
#include "open3d/pipelines/registration/FastGlobalRegistration.h"
#include "open3d/utility/Optional.h"

namespace open3d {

namespace geometry {
class PointCloud;
}

namespace pipelines {
namespace registration {

class Feature;
class RegistrationResult;

}  // namespace registration
}  // namespace pipelines
}  // namespace open3d



namespace stitcher {
/// \brief Fast Global Registration based on a given set of correspondences.
///
/// \param source The source point cloud.
/// \param target The target point cloud.
/// \param corres Correspondence indices between source and target point clouds.
/// \param option FGR options
open3d::pipelines::registration::RegistrationResult
FastGlobalRegistrationBasedOnCorrespondence(
        const open3d::geometry::PointCloud &source,
        const open3d::geometry::PointCloud &target,
        const open3d::pipelines::registration::CorrespondenceSet &corres,
        const open3d::pipelines::registration::FastGlobalRegistrationOption
                &option = open3d::pipelines::registration::
                        FastGlobalRegistrationOption());

/// \brief Fast Global Registration based on a given set of FPFH features.
///
/// \param source The source point cloud.
/// \param target The target point cloud.
/// \param corres Correspondence indices between source and target point clouds.
/// \param option FGR options
open3d::pipelines::registration::RegistrationResult
FastGlobalRegistrationBasedOnFeatureMatching(
        const open3d::geometry::PointCloud &source,
        const open3d::geometry::PointCloud &target,
        const open3d::pipelines::registration::Feature &source_feature,
        const open3d::pipelines::registration::Feature &target_feature,
        const open3d::pipelines::registration::FastGlobalRegistrationOption
                &option = open3d::pipelines::registration::
                        FastGlobalRegistrationOption());

}  // namespace stitcher
