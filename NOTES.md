### build history


- Windows build:
  * VS 2022
  * CUDA 12.4 & 12.5.40
  * Open3D 0.18 commit 525c4e6 (2024/06/04)

```
# debug build
mkdir build_debug
cmake -B .\build_debug\ -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Debug -DBUILD_PYTHON_MODULE=OFF -DBUILD_CUDA_MODULE=OFF -DCMAKE_INSTALL_PREFIX="C:/DevTools/open3d_lib/open3d_dbg"
cmake --build build_debug --config Debug --parallel 8 --target ALL_BUILD (build)
cmake --build build_debug --config Debug --target INSTALL (install)

# release build
mkdir build_release
cmake -B .\build_release\ -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_MODULE=OFF -DBUILD_CUDA_MODULE=OFF -DBUILD_BENCHMARKS=ON -DBUILD_UNIT_TESTS=ON -DCMAKE_INSTALL_PREFIX="C:/DevTools/open3d_lib/open3d_rel"
cmake --build build_release --config Release --parallel 16 --target ALL_BUILD (build)
cmake --build build_release --config Release --target INSTALL (install)
```


- Linux(WSL Ubuntu) build:
  * gcc/g++ 11.4.0
  * CUDA 12.4 & 12.5.40
  * Open3D 0.18 commit 525c4e6 (2024/06/04)

```
# install dependencies(Only needed for Ubuntu)
# see https://www.open3d.org/docs/release/compilation.html#install-dependencies
util/install_deps_ubuntu.sh

# 1. debug build
mkdir build_debug
cd build_debug

cmake ..  -DCMAKE_BUILD_TYPE=Debug -DBUILD_PYTHON_MODULE=OFF -DBUILD_CUDA_MODULE=ON -DCMAKE_INSTALL_PREFIX="/home/xzhao/workdir/installed/open3d_dbg"
make -j8 (build)
make install (install)

*** The same errors to the Window build
*** CMake Error at cmake/Findthrust.cmake:18

```

- Options:
  - BUILD_CUDA_MODULE=ON May cause compile errors! we can turn it off if it happens. This may be caused by thrust [issue](https://github.com/isl-org/Open3D/issues/6743)
  - ENABLE_CACHED_CUDA_MANAGER=ON Causes CUDA runtime error on Windows(See CMakeLists.txt), so we turn it off
  - BUILD_BENCHMARKS=ON: Only for relase build. src is in the folder cpp/benchmarks
  - BUILD_UNIT_TESTS=ON: Only for relase build. src is in the folder cpp/tests
  - By default DEVELOPER_BUILD=ON in the CMakeLists.txt

- [Build from source guide](https://www.open3d.org/docs/release/compilation.html#)