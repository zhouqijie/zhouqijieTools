#pragma once

#include <cstdint>

#if defined(_WIN32)
#if defined(RECONSTRUCTION_NATIVE_EXPORTS)
#define RT_API extern "C" __declspec(dllexport)
#else
#define RT_API extern "C" __declspec(dllimport)
#endif
#define RT_CALL __cdecl
#else
#define RT_API extern "C"
#define RT_CALL
#endif

enum RT_Status : std::int32_t {
    RT_SUCCESS = 0,
    RT_INVALID_ARGUMENT = 1,
    RT_DEGENERATE_GEOMETRY = 2,
    RT_INITIALIZATION_FAILED = 3,
    RT_OPTIMIZATION_FAILED = 4,
    RT_AMBIGUOUS_SOLUTION = 5,
    RT_HIGH_REPROJECTION_ERROR = 6,
    RT_INTERNAL_ERROR = 7,
};

struct RT_CameraInput {
    std::int32_t width;
    std::int32_t height;
    double min_vertical_fov_degrees;
    double max_vertical_fov_degrees;
};

struct RT_Observation {
    double x;
    double y;
};

struct RT_SolveOptions {
    std::int32_t scale_point_id_a;
    std::int32_t scale_point_id_b;
    double known_scale_distance;
    double max_normalized_reprojection_error;
    std::int32_t random_seed;
    std::int32_t max_candidates;
};

struct RT_CameraOutput {
    double focal_length_pixels;
    double horizontal_fov_degrees;
    double vertical_fov_degrees;
    double position[3];
    double rotation_xyzw[4];
    double reprojection_rms_pixels;
};

struct RT_PointOutput {
    std::int32_t id;
    double position[3];
    double reprojection_rms_pixels;
};

struct RT_SolveReport {
    std::int32_t status;
    std::int32_t point_count;
    std::int32_t inlier_count;
    double normalized_reprojection_rms;
    double median_triangulation_angle_degrees;
    double applied_scale;
};

RT_API const char *RT_CALL RT_GetVersion();

// observations are camera-major: observations[camera_index * point_count + point_index].
RT_API std::int32_t RT_CALL RT_SolveThreeView(
    const RT_CameraInput *cameras,
    const std::int32_t *point_ids,
    const RT_Observation *observations,
    std::int32_t point_count,
    const RT_SolveOptions *options,
    RT_CameraOutput *camera_outputs,
    RT_PointOutput *point_outputs,
    RT_SolveReport *report,
    char *error_buffer,
    std::int32_t error_buffer_capacity);
