cmake_minimum_required(VERSION 3.24)

project(ReconstructionNative VERSION 1.0.0 LANGUAGES CXX)

include(FetchContent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3147391d946bb4b6c68edd901f2add6ac1f31f8c
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(eigen)
set(Eigen3_DIR "${eigen_BINARY_DIR}" CACHE PATH "" FORCE)

set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BUILD_DOCUMENTATION OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(MINIGLOG ON CACHE BOOL "" FORCE)
set(MINIGLOG_MAX_LOG_LEVEL -1 CACHE STRING "" FORCE)
set(GFLAGS OFF CACHE BOOL "" FORCE)
set(SUITESPARSE OFF CACHE BOOL "" FORCE)
set(CXSPARSE OFF CACHE BOOL "" FORCE)
set(EIGENSPARSE OFF CACHE BOOL "" FORCE)
set(USE_CUDA OFF CACHE BOOL "" FORCE)
set(LAPACK OFF CACHE BOOL "" FORCE)
set(SCHUR_SPECIALIZATIONS OFF CACHE BOOL "" FORCE)
set(PROVIDE_UNINSTALL_TARGET OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    ceres
    GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver.git
    GIT_TAG 85331393dc0dff09f6fb9903ab0c4bfa3e134b01
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(ceres)

set(PYTHON_PACKAGE OFF CACHE BOOL "" FORCE)
set(WITH_BENCHMARK OFF CACHE BOOL "" FORCE)
set(MARCH_NATIVE OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    poselib
    GIT_REPOSITORY https://github.com/PoseLib/PoseLib.git
    GIT_TAG 7e9f5f53372e43f89655040d4dfc4a00e5ace11c
)
FetchContent_MakeAvailable(poselib)

add_library(ReconstructionNative SHARED
    include/reconstruction_native.h
    src/reconstruction_native.cpp
)

target_include_directories(ReconstructionNative
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(ReconstructionNative
    PRIVATE
        PoseLib::PoseLib
        Ceres::ceres
        Eigen3::Eigen
)

target_compile_definitions(ReconstructionNative PRIVATE RECONSTRUCTION_NATIVE_EXPORTS)

if(MSVC)
    target_compile_options(ReconstructionNative PRIVATE
        /W4 /EHsc /permissive- /bigobj
        $<$<CONFIG:Release>:/Zi>
    )
    target_link_options(ReconstructionNative PRIVATE
        $<$<CONFIG:Release>:/DEBUG:FULL>
        $<$<CONFIG:Release>:/OPT:REF>
        $<$<CONFIG:Release>:/OPT:ICF>
    )
endif()

set_target_properties(ReconstructionNative PROPERTIES
    OUTPUT_NAME "ReconstructionNative"
)

option(RECONSTRUCTION_BUILD_TESTS "Build the native synthetic tests." ON)
if(RECONSTRUCTION_BUILD_TESTS)
    enable_testing()
    add_executable(ReconstructionNativeTests
        tests/reconstruction_native_tests.cpp
    )
    target_link_libraries(ReconstructionNativeTests PRIVATE ReconstructionNative Eigen3::Eigen)
    add_test(NAME ReconstructionNativeSynthetic COMMAND ReconstructionNativeTests)
endif()
