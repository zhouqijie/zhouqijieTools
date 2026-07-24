#include "reconstruction_native.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

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
    std::array<SyntheticCamera, 3> cameras{{
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
    }};

    std::mt19937 generator(42);
    std::uniform_real_distribution<double> x_distribution(-0.9, 0.9);
    std::uniform_real_distribution<double> y_distribution(-0.65, 0.65);
    std::uniform_real_distribution<double> z_distribution(4.0, 7.0);
    std::normal_distribution<double> noise(0.0, 0.1);

    constexpr int point_count = 30;
    std::vector<Eigen::Vector3d> points;
    std::vector<std::int32_t> ids;
    points.reserve(point_count);
    ids.reserve(point_count);
    for (int index = 0; index < point_count; ++index) {
        points.emplace_back(x_distribution(generator), y_distribution(generator), z_distribution(generator));
        ids.push_back(100 + index);
    }

    std::array<RT_CameraInput, 3> camera_inputs{};
    std::vector<RT_Observation> observations(3 * point_count);
    for (int camera_index = 0; camera_index < 3; ++camera_index) {
        camera_inputs[camera_index] = {
            cameras[camera_index].width,
            cameras[camera_index].height,
            15.0,
            120.0,
        };
        for (int point_index = 0; point_index < point_count; ++point_index) {
            Eigen::Vector2d projected = Project(cameras[camera_index], points[point_index]);
            projected.x() += noise(generator);
            projected.y() += noise(generator);
            observations[camera_index * point_count + point_index] = {projected.x(), projected.y()};
        }
    }

    int scale_index_a = 0;
    int scale_index_b = 1;
    double known_distance = 0.0;
    for (int first = 0; first < point_count; ++first) {
        for (int second = first + 1; second < point_count; ++second) {
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
    std::array<RT_CameraOutput, 3> camera_outputs{};
    std::vector<RT_PointOutput> point_outputs(point_count);
    RT_SolveReport report{};
    char error[2048]{};
    const int status = RT_SolveThreeView(
        camera_inputs.data(),
        ids.data(),
        observations.data(),
        point_count,
        &options,
        camera_outputs.data(),
        point_outputs.data(),
        &report,
        error,
        static_cast<std::int32_t>(sizeof(error)));

    if (status != RT_SUCCESS) {
        std::cerr << "Synthetic solve failed: " << error << '\n';
        return 1;
    }

    for (int camera_index = 0; camera_index < 3; ++camera_index) {
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
    for (int camera_index = 0; camera_index < 3; ++camera_index) {
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
        if ((actual - expected).norm() > position_tolerance) {
            std::cerr << "Point position error exceeds 2% of the reference distance at index "
                      << point_index << ": error=" << (actual - expected).norm()
                      << ", tolerance=" << position_tolerance << ".\n";
            return 6;
        }
    }

    const auto expect_failure = [&](
                                    const char *name,
                                    const std::array<RT_CameraInput, 3> &failure_cameras,
                                    const std::vector<RT_Observation> &failure_observations,
                                    const RT_SolveOptions &failure_options) {
        std::array<RT_CameraOutput, 3> failure_camera_outputs{};
        std::vector<RT_PointOutput> failure_point_outputs(point_count);
        RT_SolveReport failure_report{};
        char failure_error[2048]{};
        const int failure_status = RT_SolveThreeView(
            failure_cameras.data(),
            ids.data(),
            failure_observations.data(),
            point_count,
            &failure_options,
            failure_camera_outputs.data(),
            failure_point_outputs.data(),
            &failure_report,
            failure_error,
            static_cast<std::int32_t>(sizeof(failure_error)));
        if (failure_status == RT_SUCCESS || failure_error[0] == '\0') {
            std::cerr << name << " did not return a clear failure.\n";
            return false;
        }
        std::cout << name << " rejected: " << failure_error << '\n';
        return true;
    };

    RT_SolveOptions wrong_scale_options = options;
    wrong_scale_options.scale_point_id_b = -123456;
    if (!expect_failure("Wrong scale ID", camera_inputs, observations, wrong_scale_options)) {
        return 7;
    }

    std::vector<RT_Observation> incorrect_matches = observations;
    for (int index = 0; index < 5; ++index) {
        std::swap(
            incorrect_matches[2 * point_count + index],
            incorrect_matches[2 * point_count + 10 + index]);
    }
    if (!expect_failure("Incorrect correspondences", camera_inputs, incorrect_matches, options)) {
        return 8;
    }

    std::vector<Eigen::Vector3d> planar_points;
    planar_points.reserve(point_count);
    for (int index = 0; index < point_count; ++index) {
        planar_points.emplace_back(x_distribution(generator), y_distribution(generator), 5.5);
    }
    std::vector<RT_Observation> planar_observations(3 * point_count);
    for (int camera_index = 0; camera_index < 3; ++camera_index) {
        for (int point_index = 0; point_index < point_count; ++point_index) {
            const Eigen::Vector2d projected = Project(cameras[camera_index], planar_points[point_index]);
            planar_observations[camera_index * point_count + point_index] = {
                projected.x(), projected.y()};
        }
    }
    if (!expect_failure("Planar points", camera_inputs, planar_observations, options)) {
        return 9;
    }

    std::array<SyntheticCamera, 3> pure_rotation_cameras = cameras;
    pure_rotation_cameras[0].rotation = Eigen::Matrix3d::Identity();
    pure_rotation_cameras[1].rotation =
        Eigen::AngleAxisd(0.04, Eigen::Vector3d::UnitY()).toRotationMatrix();
    pure_rotation_cameras[2].rotation =
        Eigen::AngleAxisd(-0.04, Eigen::Vector3d::UnitY()).toRotationMatrix();
    std::vector<RT_Observation> pure_rotation_observations(3 * point_count);
    for (int camera_index = 0; camera_index < 3; ++camera_index) {
        pure_rotation_cameras[camera_index].center.setZero();
        for (int point_index = 0; point_index < point_count; ++point_index) {
            const Eigen::Vector2d projected =
                Project(pure_rotation_cameras[camera_index], points[point_index]);
            pure_rotation_observations[camera_index * point_count + point_index] = {
                projected.x(), projected.y()};
        }
    }
    if (!expect_failure("Pure rotation", camera_inputs, pure_rotation_observations, options)) {
        return 10;
    }

    std::array<SyntheticCamera, 3> tiny_baseline_cameras = pure_rotation_cameras;
    std::vector<RT_Observation> tiny_baseline_observations(3 * point_count);
    for (int camera_index = 0; camera_index < 3; ++camera_index) {
        tiny_baseline_cameras[camera_index].center = cameras[camera_index].center * 1e-5;
        for (int point_index = 0; point_index < point_count; ++point_index) {
            const Eigen::Vector2d projected =
                Project(tiny_baseline_cameras[camera_index], points[point_index]);
            tiny_baseline_observations[camera_index * point_count + point_index] = {
                projected.x(), projected.y()};
        }
    }
    if (!expect_failure("Tiny baseline", camera_inputs, tiny_baseline_observations, options)) {
        return 11;
    }

    std::array<RT_CameraInput, 3> boundary_cameras = camera_inputs;
    for (int camera_index = 0; camera_index < 3; ++camera_index) {
        boundary_cameras[camera_index].min_vertical_fov_degrees =
            cameras[camera_index].vertical_fov - 0.1;
        boundary_cameras[camera_index].max_vertical_fov_degrees =
            cameras[camera_index].vertical_fov + 0.1;
    }
    if (!expect_failure("FOV boundary", boundary_cameras, observations, options)) {
        return 12;
    }

    std::cout << "Synthetic solve passed. RMS=" << report.normalized_reprojection_rms
              << ", median angle=" << report.median_triangulation_angle_degrees << '\n';
    return 0;
}

} // namespace

int main() {
    return RunSyntheticSolve();
}
