#include "test_fgr.h"
#include "FastGlobalRegistration.h"

using namespace open3d;

const std::string outdir{"./stitcher/test_output"};

/// ----------------------------------------------------------------------------------------------------------------
/// preprocessing: read -> downsampling -> compute normals -> FPFH feature
std::tuple<std::shared_ptr<geometry::PointCloud>, 
	std::shared_ptr<geometry::PointCloud>, 
	std::shared_ptr<pipelines::registration::Feature>>
PreprocessPointCloud(const char *file_name,
                     const double voxel_size,
                     const double search_r,
                     bool down_sample,
                     const int max_nn) {
    auto pcd = open3d::io::CreatePointCloudFromFile(file_name);
    auto pcd_down = pcd;
    if (down_sample) {
        auto pcd_down = pcd->VoxelDownSample(voxel_size);
    }
    
    pcd_down->EstimateNormals(
            open3d::geometry::KDTreeSearchParamHybrid(search_r, max_nn));
    
    const double fpfh_r = 2 * search_r;
    auto pcd_fpfh = pipelines::registration::ComputeFPFHFeature(
            *pcd_down,
            open3d::geometry::KDTreeSearchParamHybrid(
                    fpfh_r, (std::max)(100, static_cast<int>(max_nn * 1.5))));
    
    return std::make_tuple(pcd, pcd_down, pcd_fpfh);
}

/// ----------------------------------------------------------------------------------------------------------------
void VisualizeRegistration(const open3d::geometry::PointCloud &source,
                           const open3d::geometry::PointCloud &target,
                           const Eigen::Matrix4d &Transformation) {
    std::shared_ptr<geometry::PointCloud> source_transformed_ptr(
            new geometry::PointCloud);
    std::shared_ptr<geometry::PointCloud> target_ptr(new geometry::PointCloud);
    *source_transformed_ptr = source;
    *target_ptr = target;
    source_transformed_ptr->Transform(Transformation);
    visualization::DrawGeometries({source_transformed_ptr, target_ptr},
                                  "Registration result");
}


/// ----------------------------------------------------------------------------------------------------------------
/*
 .\build\bin\Release\stitcher.exe C:/workdir/data_scan/20240328T_PatternTest_Meca_Prime_Crown/test_stop_resume/20240328T212826/output/filt_res_2000_2300.ply C:/workdir/data_scan/20240328T_PatternTest_Meca_Prime_Crown/test_stop_resume/20240328T212826/output/stitch_res_1_4000.ply
 
 .\build\bin\Release\stitcher.exe C:/workdir/data_scan/20240328T_PatternTest_Meca_Prime_Crown/test_stop_resume/20240328T212826/output/stitch_src_30.ply C:/workdir/data_scan/20240328T_PatternTest_Meca_Prime_Crown/test_stop_resume/20240328T212826/output/stitch_res_1_4000.ply
 */
void RunFastGlobalRegistration(const char *file_src,
                               const char *file_tgt,
                               double voxel_size,
                               double search_r,
                               bool down_sample,
                               const FgrParams& fgr_params) {
    // Prepare input
    std::shared_ptr<geometry::PointCloud> source, source_down, target,
            target_down;
    std::shared_ptr<pipelines::registration::Feature> source_fpfh, target_fpfh;
    const int max_nn = 500;
    std::tie(source, source_down, source_fpfh) = PreprocessPointCloud(
            file_src, voxel_size, search_r, down_sample, max_nn);
    std::tie(target, target_down, target_fpfh) = PreprocessPointCloud(
            file_tgt, voxel_size, search_r, down_sample, max_nn);

    // save point cloud with normals for visualization and debug
    {
        utility::ScopeTimer scope_timer("Save point clouds as .ply files");
        const std::string output_file0{outdir + "/source_down.ply"};
        io::WritePointCloud(output_file0, *source_down);
        const std::string output_file1{outdir + "/target_down.ply"};
        io::WritePointCloud(output_file1, *target_down);
    }
    
    utility::Timer timer;

    // call FGR, return pipelines::registration::RegistrationResult    
    pipelines::registration::FastGlobalRegistrationOption fgr_option(
            fgr_params.division_factor, fgr_params.use_absolute_scale,
            fgr_params.decrease_mu,
            fgr_params.maximum_correspondence_distance,
            fgr_params.iteration_number, fgr_params.tuple_scale,
            fgr_params.max_tuples);

    // Call Open3D native FGR
    timer.Start();
    auto registration_result = pipelines::registration::
            FastGlobalRegistrationBasedOnFeatureMatching(
                    *source_down, *target_down, *source_fpfh, *target_fpfh,
                    fgr_option);
    timer.Stop();
    utility::LogInfo(
            "FastGlobalRegistration (Original) takes {} milliseconds.\n",
            timer.GetDurationInMillisecond());

    // Call Modified FGR
    timer.Start();
    auto registration_result_m =
            stitcher::FastGlobalRegistrationBasedOnFeatureMatching(
                    *source_down, *target_down, *source_fpfh, *target_fpfh,
                    fgr_option);
    timer.Stop();
    utility::LogInfo("FastGlobalRegistration (Modified) takes {} milliseconds.\n",
                     timer.GetDurationInMillisecond());


    // save the transformed source before visualization
    std::shared_ptr<geometry::PointCloud> source_transformed_ptr(
            new geometry::PointCloud);
    *source_transformed_ptr = *source;
    source_transformed_ptr->Transform(registration_result_m.transformation_);
    std::string output_file{outdir + "./fgr_aligned.ply"};
    io::WritePointCloud(output_file, *source_transformed_ptr);

    VisualizeRegistration(*source, *target,
                          registration_result_m.transformation_);
    std::cout << "===>The end of function " << __func__ << std::endl;
}