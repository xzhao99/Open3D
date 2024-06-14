#include <iostream>

#include "open3d/Open3D.h"
#include "test_fgr.h"

void PrintHelp() {
    using namespace open3d;

    // clang-format off
    utility::LogInfo("Usage:");
    utility::LogInfo("    > Source PointCloud [pointcloud_filename]");
    utility::LogInfo("    > Target PointCloud [pointcloud_filename]");
    // clang-format on
    utility::LogInfo("");
}


int main(int argc, char *argv[]) {
	using namespace open3d;
    utility::LogInfo("Test: print Open3D version:");
    PrintOpen3DVersion();

	utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

    if (argc != 3 ||
        utility::ProgramOptionExistsAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }
	
	const double voxel_size = 0.1;
    const double search_r = 5;  // used to compute Normal and Feature
    const double dist_multiplier = 5;
    const bool down_sample = false;
    FgrParams fgr_params;
    fgr_params.use_absolute_scale = true;
    fgr_params.decrease_mu = true;
    fgr_params.maximum_correspondence_distance = voxel_size * dist_multiplier;
    fgr_params.iteration_number = 64;
    fgr_params.max_tuples = 1000;
    fgr_params.tuple_test = true;
    fgr_params.tuple_scale = 0.95;

    // [Open3D INFO] FastGlobalRegistration takes 3219.06 milliseconds.(Open3D)
    RunFastGlobalRegistration(argv[1], argv[2], voxel_size, search_r,
                              down_sample, fgr_params);

	return 1;
}