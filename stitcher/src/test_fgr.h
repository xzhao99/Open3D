#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <memory>

#include "open3d/Open3D.h"


/// ----------------------------------------------------------------------------------------------------------------
struct FgrParams {
    double division_factor = 1.4;
    bool use_absolute_scale = false;
    bool decrease_mu = true;
    double maximum_correspondence_distance{0.025};
    int iteration_number{64};
    double tuple_scale{0.95};
    int max_tuples{1000};
    bool tuple_test{true};
};


/// ----------------------------------------------------------------------------------------------------------------
/// preprocessing: read -> downsampling -> compute normals -> FPFH feature
std::tuple<std::shared_ptr<open3d::geometry::PointCloud>,
           std::shared_ptr<open3d::geometry::PointCloud>,
           std::shared_ptr<open3d::pipelines::registration::Feature>>
PreprocessPointCloud(const char *file_name,
                     const double voxel_size,
                     const double search_r,
                     bool down_sample,
                     const int max_nn);


/// ----------------------------------------------------------------------------------------------------------------
void VisualizeRegistration(const open3d::geometry::PointCloud &source,
                           const open3d::geometry::PointCloud &target,
                           const Eigen::Matrix4d &Transformation);

/// ----------------------------------------------------------------------------------------------------------------
void RunFastGlobalRegistration(const char *file_src,
                               const char *file_tgt,
                               double voxel_size,
                               double search_r,
                               bool down_sample,
                               const FgrParams &fgr_params);

