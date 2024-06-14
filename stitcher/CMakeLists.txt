# CMakeLists.txt for stitcher
# ref: ./examples/cpp/CMakeLists.txt

set(STITCHER_BIN_DIR "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/stitcher")
set(EXE_NAME stitcher)
message(STATUS "==================Building stitcher...==================")
message(STATUS "STITCHER_BIN_DIR: ${STITCHER_BIN_DIR}")
message(STATUS "ARGN: ${ARGN}")

# find OpenMP
find_package(OpenMP)
if(OpenMP_FOUND)
    message(STATUS "OpenMP is found!")
endif()

# add target
add_executable(${EXE_NAME} main.cpp)


target_sources(${EXE_NAME} 
    PRIVATE 
    "src/FastGlobalRegistration.cpp"
    "src/test_fgr.cpp"
    PUBLIC
    "src/FastGlobalRegistration.h"
    "src/test_fgr.h"
)


set_target_properties(${EXE_NAME} PROPERTIES
        FOLDER "stitcher/"
        RUNTIME_OUTPUT_DIRECTORY "${STITCHER_BIN_DIR}"
    )


target_include_directories(${EXE_NAME} PRIVATE "./src")
target_link_libraries(${EXE_NAME} PRIVATE Open3D::Open3D Open3D::3rdparty_openmp ${ARGN})

if(WIN32)
    target_compile_options(${EXE_NAME} PRIVATE "-openmp:llvm")
endif()