#include "reconstruction_native.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

struct SyntheticCamera {
    int width;
    int height;
    double vertical_fov;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d center;
};

double FocalFromFov(const int height, const double fov) {
    return height / (2.0 * std::tan(fov * kPi / 360.0));
}

Eigen::Vector2d Project(const SyntheticCamera &camera, const Eigen::Vector3d &point) {
    const Eigen::Vector3d local = camera.rotation * (point - camera.center);
    const double focal = FocalFromFov(camera.height, camera.vertical_fov);
    return {
        focal * local.x() / local.z() + camera.width * 0.5,
        focal * local.y() / local.z() + camera.height * 0.5,
    };
}

int RunSyntheticSolve() {
    constexpr int camera_count = 6;
    std::array<SyntheticCamera, camera_count> cameras{{
        {4000, 3000, 52.0, Eigen::Matrix3d::Identity(), {0.0, 0.0, 0.0}},
        {1920,
         1080,
         68.0,
         Eigen::AngleAxisd(0.18, Eigen::Vector3d::UnitY()).toRotationMatrix(),
         {1.15, 0.08, 0.18}},
        {3024,
         4032,
         44.0,
         (Eigen::AngleAxisd(-0.15, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(0.04, Eigen::Vector3d::UnitX()))
             .toRotationMatrix(),
         {-0.8, -0.12, 0.35}},
        {2560,
         1440,
         58.0,
         (Eigen::AngleAxisd(0.11, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(-0.03, Eigen::Vector3d::UnitX()))
             .toRotationMatrix(),
         {0.45, 0.25, -0.25}},
        {2048,
         1536,
         63.0,
         (Eigen::AngleAxisd(-0.09, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(0.025, Eigen::Vector3d::UnitX()))
             .toRotationMatrix(),
         {-0.35, 0.2, -0.15}},
        {2880,
         1620,
         61.0,
         (Eigen::AngleAxisd(0.07, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(0.035, Eigen::Vector3d::UnitX()))
             .toRotationMatrix(),
         {0.7, -0.18, 0.05}},
    }};

    std::mt19937 generator(42);
    std::uniform_real_distribution<double> x_distribution(-0.9, 0.9);
    std::uniform_real_distribution<double> y_distribution(-0.65, 0.65);
    std::uniform_real_distribution<double> z_distribution(4.0, 7.0);
    std::normal_distribution<double> noise(0.0, 0.1);

    constexpr int base_point_count = 30;
    constexpr int point_count = 31;
    constexpr int line_count = 2;
    std::vector<Eigen::Vector3d> points;
    std::vector<std::int32_t> ids;
    points.reserve(point_count);
    ids.reserve(point_count);
    for (int index = 0; index < point_count; ++index) {
        points.emplace_back(x_distribution(generator), y_distribution(generator), z_distribution(generator));
        ids.push_back(100 + index);
    }

    const std::array<Eigen::Vector3d, line_count> line_points{{
        {-0.2, 0.1, 5.2},
        {0.3, -0.25, 5.8},
    }};
    const std::array<Eigen::Vector3d, line_count> line_directions{{
        Eigen::Vector3d(0.9, 0.25, 0.15).normalized(),
        Eigen::Vector3d(-0.15, 0.85, 0.2).normalized(),
    }};
    const std::array<std::int32_t, line_count> line_ids{{20, 21}};

    std::array<RT_CameraInput, camera_count> camera_inputs{};
    std::vector<RT_Observation> observations(camera_count * point_count);
    std::vector<std::uint8_t> observation_visibility(camera_count * point_count, 0);
    std::vector<double> observation_confidences(camera_count * point_count, 1.0);
    std::vector<RT_LineObservation> line_observations(camera_count * line_count);
    std::vector<std::uint8_t> line_observation_visibility(camera_count * line_count, 0);
    for (int camera_index = 0; camera_index < camera_count; ++camera_index) {
        camera_inputs[camera_index] = {
            cameras[camera_index].width,
            cameras[camera_index].height,
            15.0,
            120.0,
            1.0,
            camera_index == 4 ? 1 : 0,
        };
        for (int point_index = 0; point_index < point_count; ++point_index) {
            Eigen::Vector2d projected = Project(cameras[camera_index], points[point_index]);
            projected.x() += noise(generator);
            projected.y() += noise(generator);
            if (camera_index == 3 && point_index == 12) {
                projected.x() += 23.0;
            }
            observations[camera_index * point_count + point_index] = {projected.x(), projected.y()};
            const bool visible_base =
                camera_index < 3
                    ? point_index < base_point_count
                    : camera_index == 3
                        ? point_index == 0 ||
                          point_index == 5 ||
                          point_index == 7 ||
                          point_index == 12 ||
                          point_index == 18 ||
                          point_index == 29
                        : camera_index == 4
                            ? point_index < base_point_count &&
                              point_index % 2 == 1
                            : point_index < base_point_count &&
                              point_index % 2 == 0;
            observation_visibility[camera_index * point_count + point_index] =
                visible_base ||
                (camera_index >= 3 && point_index == base_point_count)
                    ? 1
                    : 0;
        }
        for (int line_index = 0; line_index < line_count; ++line_index) {
            const double start_offset = -1.2 - camera_index * 0.25 + line_index * 0.1;
            const double end_offset = 1.1 + camera_index * 0.35 + line_index * 0.15;
            const Eigen::Vector2d start = Project(
                cameras[camera_index],
                line_points[line_index] + start_offset * line_directions[line_index]);
            const Eigen::Vector2d end = Project(
                cameras[camera_index],
                line_points[line_index] + end_offset * line_directions[line_index]);
            line_observations[camera_index * line_count + line_index] = {
                start.x(),
                start.y(),
                end.x(),
                end.y(),
                line_index == 0 ? 1.0 : 0.7,
            };
            line_observation_visibility[camera_index * line_count + line_index] =
                camera_index < 3 ||
                camera_index == 3 ||
                (camera_index == 4 && line_index == 1) ||
                (camera_index == 5 && line_index == 0)
                    ? 1
                    : 0;
        }
    }
    observation_confidences[3 * point_count] = 0.4;
    observation_confidences[3 * point_count + 12] = 0.1;

    int scale_index_a = 0;
    int scale_index_b = 1;
    double known_distance = 0.0;
    for (int first = 0; first < base_point_count; ++first) {
        for (int second = first + 1; second < base_point_count; ++second) {
            const double distance = (points[first] - points[second]).norm();
            if (distance > known_distance) {
                known_distance = distance;
                scale_index_a = first;
                scale_index_b = second;
            }
        }
    }
    RT_SolveOptions options{
        ids[scale_index_a],
        ids[scale_index_b],
        known_distance,
        8.0,
        42,
        12,
    };
    std::array<RT_CameraOutput, camera_count> camera_outputs{};
    std::vector<RT_PointOutput> point_outputs(point_count);
    std::array<RT_LineOutput, line_count> line_outputs{};
    RT_SolveReport report{};
    char error[2048]{};
    const int status = RT_SolveMultiView(
        camera_inputs.data(),
        camera_count,
        ids.data(),
        observations.data(),
        observation_visibility.data(),
        observation_confidences.data(),
        point_count,
        base_point_count,
        line_ids.data(),
        line_observations.data(),
        line_observation_visibility.data(),
        line_count,
        &options,
        camera_outputs.data(),
        point_outputs.data(),
        line_outputs.data(),
        &report,
        error,
        static_cast<std::int32_t>(sizeof(error)));

    if (status != RT_SUCCESS) {
        std::cerr << "Synthetic solve failed: " << error << '\n';
        return 1;
    }

    // 验证附加机位的错误参考线会输出具体 Camera 和 Line。
    std::vector<RT_LineObservation> incorrect_line_observations = line_observations;
    RT_LineObservation &incorrect_line =
        incorrect_line_observations[5 * line_count];
    incorrect_line.start_y += 300.0;
    incorrect_line.end_y += 300.0;
    incorrect_line.confidence = 0.1;
    std::array<RT_CameraOutput, camera_count> line_failure_camera_outputs{};
    std::vector<RT_PointOutput> line_failure_point_outputs(point_count);
    std::array<RT_LineOutput, line_count> line_failure_line_outputs{};
    RT_SolveReport line_failure_report{};
    char line_failure_error[8192]{};
    const int line_failure_status = RT_SolveMultiView(
        camera_inputs.data(),
        camera_count,
        ids.data(),
        observations.data(),
        observation_visibility.data(),
        observation_confidences.data(),
        point_count,
        base_point_count,
        line_ids.data(),
        incorrect_line_observations.data(),
        line_observation_visibility.data(),
        line_count,
        &options,
        line_failure_camera_outputs.data(),
        line_failure_point_outputs.data(),
        line_failure_line_outputs.data(),
        &line_failure_report,
        line_failure_error,
        static_cast<std::int32_t>(sizeof(line_failure_error)));
    const std::string line_failure_message(line_failure_error);
    if (line_failure_status != RT_HIGH_REPROJECTION_ERROR ||
        line_failure_message.find("Camera 5 / Line 20") == std::string::npos) {
        std::cerr << "Detailed line diagnostics failed: " << line_failure_error << '\n';
        return 17;
    }
    std::cout << "Detailed line diagnostics passed: " << line_failure_error << '\n';

    std::array<RT_CameraOutput, camera_count> point_only_camera_outputs{};
    std::vector<RT_PointOutput> point_only_point_outputs(point_count);
    RT_SolveReport point_only_report{};
    char point_only_error[2048]{};
    const int point_only_status = RT_SolveMultiView(
        camera_inputs.data(),
        camera_count,
        ids.data(),
        observations.data(),
        observation_visibility.data(),
        observation_confidences.data(),
        point_count,
        base_point_count,
        nullptr,
        nullptr,
        nullptr,
        0,
        &options,
        point_only_camera_outputs.data(),
        point_only_point_outputs.data(),
        nullptr,
        &point_only_report,
        point_only_error,
        static_cast<std::int32_t>(sizeof(point_only_error)));
    if (point_only_status != RT_SUCCESS || point_only_report.line_count != 0) {
        std::cerr << "Point-only solve regressed: " << point_only_error << '\n';
        return 16;
    }

    for (int camera_index = 0; camera_index < camera_count; ++camera_index) {
        const double error_degrees =
            std::abs(camera_outputs[camera_index].vertical_fov_degrees - cameras[camera_index].vertical_fov);
        std::cout << "Camera " << camera_index
                  << " FOV=" << camera_outputs[camera_index].vertical_fov_degrees
                  << " error=" << error_degrees << '\n';
        if (error_degrees > 2.0) {
            std::cerr << "FOV error exceeds 2 degrees.\n";
            return 2;
        }
    }

    if (report.normalized_reprojection_rms > 1.5) {
        std::cerr << "Normalized RMS exceeds 1.5 pixels: "
                  << report.normalized_reprojection_rms << '\n';
        return 3;
    }
    if (report.line_count != line_count || report.normalized_line_rms > 0.5) {
        std::cerr << "Line reconstruction RMS is too high: "
                  << report.normalized_line_rms << '\n';
        return 14;
    }
    if (std::abs(
            (Eigen::Vector3d(point_outputs[scale_index_a].position) -
             Eigen::Vector3d(point_outputs[scale_index_b].position))
                    .norm() -
            known_distance) > 1e-5) {
        std::cerr << "Known-distance scale was not applied.\n";
        return 4;
    }

    const Eigen::Matrix3d coordinate_flip =
        (Eigen::Matrix3d() << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0).finished();
    const double position_tolerance = known_distance * 0.02;
    for (int camera_index = 0; camera_index < camera_count; ++camera_index) {
        const Eigen::Vector3d actual(camera_outputs[camera_index].position);
        const Eigen::Vector3d expected = coordinate_flip * cameras[camera_index].center;
        if ((actual - expected).norm() > position_tolerance) {
            std::cerr << "Camera position error exceeds 2% of the reference distance.\n";
            return 5;
        }
    }
    for (int point_index = 0; point_index < point_count; ++point_index) {
        const Eigen::Vector3d actual(point_outputs[point_index].position);
        const Eigen::Vector3d expected = coordinate_flip * points[point_index];
        const double point_tolerance =
            point_index < base_point_count
                ? position_tolerance
                : known_distance * 0.025;
        if ((actual - expected).norm() > point_tolerance) {
            std::cerr << "Point position error exceeds its tolerance at index "
                      << point_index << ": error=" << (actual - expected).norm()
                      << ", tolerance=" << point_tolerance << ".\n";
            return 6;
        }
    }
    for (int line_index = 0; line_index < line_count; ++line_index) {
        const Eigen::Vector3d actual_point(line_outputs[line_index].point);
        const Eigen::Vector3d actual_direction =
            Eigen::Vector3d(line_outputs[line_index].direction).normalized();
        const Eigen::Vector3d expected_point = coordinate_flip * line_points[line_index];
        const Eigen::Vector3d expected_direction =
            (coordinate_flip * line_directions[line_index]).normalized();
        const double point_to_line =
            (actual_point - expected_point).cross(expected_direction).norm();
        if (std::abs(actual_direction.dot(expected_direction)) < 0.999 ||
            point_to_line > position_tolerance) {
            std::cerr << "3D line error exceeds tolerance at index " << line_index
                      << ": point error=" << point_to_line << ".\n";
            return 15;
        }
    }

    const std::array<RT_CameraInput, 3> base_camera_inputs{{
        camera_inputs[0],
        camera_inputs[1],
        camera_inputs[2],
    }};
    const std::vector<std::int32_t> base_ids(
        ids.begin(),
        ids.begin() + base_point_count);
    std::vector<RT_Observation> base_observations(3 * base_point_count);
    for (int camera_index = 0; camera_index < 3; ++camera_index) {
        std::copy_n(
            observations.begin() + camera_index * point_count,
            base_point_count,
            base_observations.begin() + camera_index * base_point_count);
    }
    const std::vector<std::uint8_t> base_visibility(3 * base_point_count, 1);
    const std::vector<double> base_confidences(3 * base_point_count, 1.0);
    const auto expect_failure = [&](
                                    const char *name,
                                    const std::array<RT_CameraInput, 3> &failure_cameras,
                                    const std::vector<RT_Observation> &failure_observations,
                                    const RT_SolveOptions &failure_options) {
        std::array<RT_CameraOutput, 3> failure_camera_outputs{};
        std::vector<RT_PointOutput> failure_point_outputs(base_point_count);
        RT_SolveReport failure_report{};
        char failure_error[2048]{};
        const int failure_status = RT_SolveMultiView(
            failure_cameras.data(),
            3,
            base_ids.data(),
            failure_observations.data(),
            base_visibility.data(),
            base_confidences.data(),
            base_point_count,
            base_point_count,
            nullptr,
            nullptr,
            nullptr,
            0,
            &failure_options,
            failure_camera_outputs.data(),
            failure_point_outputs.data(),
            nullptr,
            &failure_report,
            failure_error,
            static_cast<std::int32_t>(sizeof(failure_error)));
        if (failure_status == RT_SUCCESS || failure_error[0] == '\0') {
            std::cerr << name << " did not return a clear failure.\n";
            return false;
        }
        if (failure_status == RT_HIGH_REPROJECTION_ERROR) {
            for (const RT_CameraOutput &camera : failure_camera_outputs) {
                if (!(camera.focal_length_pixels > 0.0)) {
                    std::cerr << name << " did not preserve its best candidate for diagnostics.\n";
                    return false;
                }
            }
        }
        std::cout << name << " rejected: " << failure_error << '\n';
        return true;
    };

    RT_SolveOptions wrong_scale_options = options;
    wrong_scale_options.scale_point_id_b = -123456;
    if (!expect_failure(
            "Wrong scale ID",
            base_camera_inputs,
            base_observations,
            wrong_scale_options)) {
        return 7;
    }

    std::vector<RT_Observation> incorrect_matches = base_observations;
    for (int index = 0; index < 5; ++index) {
        std::swap(
            incorrect_matches[2 * base_point_count + index],
            incorrect_matches[2 * base_point_count + 10 + index]);
    }
    if (!expect_failure("Incorrect correspondences", base_camera_inputs, incorrect_matches, options)) {
        return 8;
    }

    std::vector<Eigen::Vector3d> planar_points;
    planar_points.reserve(base_point_count);
    for (int index = 0; index < base_point_count; ++index) {
        planar_points.emplace_back(x_distribution(generator), y_distribution(generator), 5.5);
    }
    std::vector<RT_Observation> planar_observations(3 * base_point_count);
    for (int camera_index = 0; camera_index < 3; ++camera_index) {
        for (int point_index = 0; point_index < base_point_count; ++point_index) {
            const Eigen::Vector2d projected = Project(cameras[camera_index], planar_points[point_index]);
            planar_observations[camera_index * base_point_count + point_index] = {
                projected.x(), projected.y()};
        }
    }
    if (!expect_failure("Planar points", base_camera_inputs, planar_observations, options)) {
        return 9;
    }

    std::array<SyntheticCamera, 3> pure_rotation_cameras{{
        cameras[0],
        cameras[1],
        cameras[2],
    }};
    pure_rotation_cameras[0].rotation = Eigen::Matrix3d::Identity();
    pure_rotation_cameras[1].rotation =
        Eigen::AngleAxisd(0.04, Eigen::Vector3d::UnitY()).toRotationMatrix();
    pure_rotation_cameras[2].rotation =
        Eigen::AngleAxisd(-0.04, Eigen::Vector3d::UnitY()).toRotationMatrix();
    std::vector<RT_Observation> pure_rotation_observations(3 * base_point_count);
    for (int camera_index = 0; camera_index < 3; ++camera_index) {
        pure_rotation_cameras[camera_index].center.setZero();
        for (int point_index = 0; point_index < base_point_count; ++point_index) {
            const Eigen::Vector2d projected =
                Project(pure_rotation_cameras[camera_index], points[point_index]);
            pure_rotation_observations[camera_index * base_point_count + point_index] = {
                projected.x(), projected.y()};
        }
    }
    if (!expect_failure("Pure rotation", base_camera_inputs, pure_rotation_observations, options)) {
        return 10;
    }

    std::array<SyntheticCamera, 3> tiny_baseline_cameras = pure_rotation_cameras;
    std::vector<RT_Observation> tiny_baseline_observations(3 * base_point_count);
    for (int camera_index = 0; camera_index < 3; ++camera_index) {
        tiny_baseline_cameras[camera_index].center = cameras[camera_index].center * 1e-5;
        for (int point_index = 0; point_index < base_point_count; ++point_index) {
            const Eigen::Vector2d projected =
                Project(tiny_baseline_cameras[camera_index], points[point_index]);
            tiny_baseline_observations[camera_index * base_point_count + point_index] = {
                projected.x(), projected.y()};
        }
    }
    if (!expect_failure("Tiny baseline", base_camera_inputs, tiny_baseline_observations, options)) {
        return 11;
    }

    std::array<RT_CameraInput, 3> boundary_cameras = base_camera_inputs;
    for (int camera_index = 0; camera_index < 3; ++camera_index) {
        boundary_cameras[camera_index].min_vertical_fov_degrees =
            cameras[camera_index].vertical_fov - 0.1;
        boundary_cameras[camera_index].max_vertical_fov_degrees =
            cameras[camera_index].vertical_fov + 0.1;
    }
    if (!expect_failure("FOV boundary", boundary_cameras, base_observations, options)) {
        return 12;
    }

    std::array<RT_CameraInput, 3> invalid_confidence_cameras = base_camera_inputs;
    invalid_confidence_cameras[2].confidence = 0.0;
    if (!expect_failure(
            "Invalid confidence",
            invalid_confidence_cameras,
            base_observations,
            options)) {
        return 13;
    }

    std::cout << "Synthetic solve passed. RMS=" << report.normalized_reprojection_rms
              << ", median angle=" << report.median_triangulation_angle_degrees << '\n';
    return 0;
}

} // namespace

int main() {
    return RunSyntheticSolve();
}
