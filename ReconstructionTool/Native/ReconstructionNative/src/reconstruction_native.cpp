#include "reconstruction_native.h"

#include <PoseLib/misc/decompositions.h>
#include <PoseLib/misc/essential.h>
#include <PoseLib/robust.h>
#include <PoseLib/solvers/p4pf.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/sphere_manifold.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace {

constexpr int kCameraCount = 3;
constexpr double kNormalizedLongSide = 1000.0;
constexpr double kPi = 3.14159265358979323846;

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using PoseArray = std::array<poselib::CameraPose, kCameraCount>;

struct NormalizedCamera {
    double width = 0.0;
    double height = 0.0;
    double pixel_scale = 1.0;
    double min_focal = 0.0;
    double max_focal = 0.0;
    double min_fov = 0.0;
    double max_fov = 0.0;
    Vec2 principal = Vec2::Zero();
};

struct PairGeometry {
    int first = 0;
    int second = 1;
    Mat3 fundamental = Mat3::Identity();
    std::vector<char> fundamental_inliers;
    std::vector<char> homography_inliers;
    double fundamental_score = std::numeric_limits<double>::max();
};

struct Candidate {
    PoseArray poses;
    std::array<double, kCameraCount> focals{};
    std::vector<Vec3> points;
    double score = std::numeric_limits<double>::max();
    double rms = std::numeric_limits<double>::max();
    double median_angle = 0.0;
    bool optimized = false;
};

struct CameraParameters {
    double rotation[3]{};
    double center[3]{};
    double log_focal = 0.0;
};

void WriteError(char *buffer, const std::int32_t capacity, const std::string &message) {
    if (buffer == nullptr || capacity <= 0) {
        return;
    }

    const auto count = static_cast<std::size_t>(capacity - 1);
    std::strncpy(buffer, message.c_str(), count);
    buffer[count] = '\0';
}

double DegreesToRadians(const double degrees) {
    return degrees * kPi / 180.0;
}

double RadiansToDegrees(const double radians) {
    return radians * 180.0 / kPi;
}

double FocalFromVerticalFov(const double height, const double fov_degrees) {
    return height / (2.0 * std::tan(DegreesToRadians(fov_degrees) * 0.5));
}

double FovFromFocal(const double extent, const double focal) {
    return RadiansToDegrees(2.0 * std::atan(extent / (2.0 * focal)));
}

Mat3 CameraMatrix(const NormalizedCamera &camera, const double focal) {
    Mat3 matrix = Mat3::Identity();
    matrix(0, 0) = focal;
    matrix(1, 1) = focal;
    matrix(0, 2) = camera.principal.x();
    matrix(1, 2) = camera.principal.y();
    return matrix;
}

Vec3 Bearing(const NormalizedCamera &camera, const double focal, const Vec2 &point) {
    return Vec3(
               (point.x() - camera.principal.x()) / focal,
               (point.y() - camera.principal.y()) / focal,
               1.0)
        .normalized();
}

Vec3 Triangulate(
    const Vec2 &point1,
    const Vec2 &point2,
    const NormalizedCamera &camera1,
    const NormalizedCamera &camera2,
    const double focal1,
    const double focal2,
    const poselib::CameraPose &pose2_from_pose1) {
    const double x1 = (point1.x() - camera1.principal.x()) / focal1;
    const double y1 = (point1.y() - camera1.principal.y()) / focal1;
    const double x2 = (point2.x() - camera2.principal.x()) / focal2;
    const double y2 = (point2.y() - camera2.principal.y()) / focal2;

    Eigen::Matrix<double, 3, 4> projection1 = Eigen::Matrix<double, 3, 4>::Zero();
    projection1.block<3, 3>(0, 0) = Mat3::Identity();
    const Eigen::Matrix<double, 3, 4> projection2 = pose2_from_pose1.Rt();

    Eigen::Matrix4d system;
    system.row(0) = x1 * projection1.row(2) - projection1.row(0);
    system.row(1) = y1 * projection1.row(2) - projection1.row(1);
    system.row(2) = x2 * projection2.row(2) - projection2.row(0);
    system.row(3) = y2 * projection2.row(2) - projection2.row(1);

    const Eigen::Vector4d homogeneous =
        system.jacobiSvd(Eigen::ComputeFullV).matrixV().col(3);
    if (std::abs(homogeneous.w()) < 1e-12) {
        return Vec3::Constant(std::numeric_limits<double>::quiet_NaN());
    }
    return homogeneous.head<3>() / homogeneous.w();
}

double ReprojectionSquared(
    const Vec3 &point,
    const Vec2 &observation,
    const NormalizedCamera &camera,
    const double focal,
    const poselib::CameraPose &pose,
    bool *positive_depth = nullptr) {
    const Vec3 camera_point = pose.apply(point);
    if (positive_depth != nullptr) {
        *positive_depth = camera_point.z() > 1e-8;
    }
    if (camera_point.z() <= 1e-8 || !camera_point.allFinite()) {
        return 1e12;
    }

    const Vec2 projected(
        focal * camera_point.x() / camera_point.z() + camera.principal.x(),
        focal * camera_point.y() / camera_point.z() + camera.principal.y());
    return (projected - observation).squaredNorm();
}

double Median(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    double result = *middle;
    if ((values.size() % 2) == 0) {
        const auto lower = std::max_element(values.begin(), middle);
        result = (*lower + result) * 0.5;
    }
    return result;
}

double ComputeMedianTriangulationAngle(const PoseArray &poses, const std::vector<Vec3> &points) {
    std::array<Vec3, kCameraCount> centers{};
    for (int camera_index = 0; camera_index < kCameraCount; ++camera_index) {
        centers[camera_index] = poses[camera_index].center();
    }

    std::vector<double> angles;
    angles.reserve(points.size());
    for (const Vec3 &point : points) {
        double maximum_angle = 0.0;
        for (int first = 0; first < kCameraCount; ++first) {
            for (int second = first + 1; second < kCameraCount; ++second) {
                const Vec3 ray1 = (point - centers[first]).normalized();
                const Vec3 ray2 = (point - centers[second]).normalized();
                const double cosine = std::clamp(ray1.dot(ray2), -1.0, 1.0);
                maximum_angle = std::max(maximum_angle, RadiansToDegrees(std::acos(cosine)));
            }
        }
        angles.push_back(maximum_angle);
    }
    return Median(std::move(angles));
}

double ScoreCandidate(
    const Candidate &candidate,
    const std::array<NormalizedCamera, kCameraCount> &cameras,
    const std::array<std::vector<Vec2>, kCameraCount> &observations,
    int *positive_count = nullptr) {
    double squared_sum = 0.0;
    int positive = 0;
    int residual_count = 0;
    for (std::size_t point_index = 0; point_index < candidate.points.size(); ++point_index) {
        for (int camera_index = 0; camera_index < kCameraCount; ++camera_index) {
            bool is_positive = false;
            squared_sum += ReprojectionSquared(
                candidate.points[point_index],
                observations[camera_index][point_index],
                cameras[camera_index],
                candidate.focals[camera_index],
                candidate.poses[camera_index],
                &is_positive);
            positive += is_positive ? 1 : 0;
            residual_count += 2;
        }
    }

    if (positive_count != nullptr) {
        *positive_count = positive;
    }
    return std::sqrt(squared_sum / std::max(1, residual_count));
}

std::vector<double> FocalSeeds(const NormalizedCamera &camera) {
    static constexpr std::array<double, 6> fov_seeds{25.0, 40.0, 55.0, 70.0, 85.0, 100.0};
    std::vector<double> focals;
    for (const double seed : fov_seeds) {
        const double clamped = std::clamp(seed, camera.min_fov, camera.max_fov);
        const double focal = FocalFromVerticalFov(camera.height, clamped);
        if (std::none_of(focals.begin(), focals.end(), [focal](const double value) {
                return std::abs(value - focal) < 1e-6;
            })) {
            focals.push_back(focal);
        }
    }
    return focals;
}

double EssentialMismatch(const Mat3 &fundamental, const Mat3 &matrix1, const Mat3 &matrix2) {
    const Mat3 essential = matrix2.transpose() * fundamental * matrix1;
    const Eigen::Vector3d singular = essential.jacobiSvd().singularValues();
    const double denominator = std::max(1e-12, singular.x() + singular.y());
    return std::abs(singular.x() - singular.y()) / denominator + singular.z() / denominator;
}

std::vector<std::pair<double, double>> EstimateFocalPairs(
    const PairGeometry &pair,
    const std::array<NormalizedCamera, kCameraCount> &cameras) {
    const auto &first_camera = cameras[pair.first];
    const auto &second_camera = cameras[pair.second];
    std::vector<std::pair<double, double>> output;

    const auto add_pair = [&](const double focal1, const double focal2) {
        if (!std::isfinite(focal1) || !std::isfinite(focal2) || focal1 <= 0.0 || focal2 <= 0.0) {
            return;
        }
        if (focal1 < first_camera.min_focal || focal1 > first_camera.max_focal ||
            focal2 < second_camera.min_focal || focal2 > second_camera.max_focal) {
            return;
        }
        for (const auto &existing : output) {
            if (std::abs(std::log(existing.first / focal1)) < 0.015 &&
                std::abs(std::log(existing.second / focal2)) < 0.015) {
                return;
            }
        }
        output.emplace_back(focal1, focal2);
    };

    try {
        const auto direct = poselib::focals_from_fundamental(
            pair.fundamental, first_camera.principal, second_camera.principal);
        add_pair(direct.first.focal(), direct.second.focal());
    } catch (...) {
    }

    const auto first_seeds = FocalSeeds(first_camera);
    const auto second_seeds = FocalSeeds(second_camera);
    for (const double first_seed : first_seeds) {
        for (const double second_seed : second_seeds) {
            try {
                const poselib::Camera first_prior(
                    "SIMPLE_PINHOLE",
                    {first_seed, first_camera.principal.x(), first_camera.principal.y()},
                    static_cast<int>(std::round(first_camera.width)),
                    static_cast<int>(std::round(first_camera.height)));
                const poselib::Camera second_prior(
                    "SIMPLE_PINHOLE",
                    {second_seed, second_camera.principal.x(), second_camera.principal.y()},
                    static_cast<int>(std::round(second_camera.width)),
                    static_cast<int>(std::round(second_camera.height)));
                const auto result = poselib::focals_from_fundamental_iterative(
                    pair.fundamental,
                    first_prior,
                    second_prior,
                    50,
                    Eigen::Vector4d(5e-4, 1.0, 5e-4, 1.0));
                add_pair(std::get<0>(result).focal(), std::get<1>(result).focal());
            } catch (...) {
            }
        }
    }

    std::sort(output.begin(), output.end(), [&](const auto &left, const auto &right) {
        const double left_score = EssentialMismatch(
            pair.fundamental,
            CameraMatrix(first_camera, left.first),
            CameraMatrix(second_camera, left.second));
        const double right_score = EssentialMismatch(
            pair.fundamental,
            CameraMatrix(first_camera, right.first),
            CameraMatrix(second_camera, right.second));
        return left_score < right_score;
    });
    if (output.size() > 10) {
        output.resize(10);
    }
    return output;
}

Mat3 ProjectToEssential(const Mat3 &essential) {
    Eigen::JacobiSVD<Mat3> svd(essential, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Mat3 left = svd.matrixU();
    Mat3 right = svd.matrixV();
    if (left.determinant() < 0.0) {
        left.col(2) *= -1.0;
    }
    if (right.determinant() < 0.0) {
        right.col(2) *= -1.0;
    }
    const auto singular = svd.singularValues();
    const double average = (singular.x() + singular.y()) * 0.5;
    return left * Eigen::Vector3d(average, average, 0.0).asDiagonal() * right.transpose();
}

bool InitializeThirdCamera(
    const int camera_index,
    const NormalizedCamera &camera,
    const std::vector<Vec2> &observations,
    const std::vector<Vec3> &points,
    const int random_seed,
    poselib::CameraPose *best_pose,
    double *best_focal,
    double *best_score) {
    if (points.size() < 4) {
        return false;
    }

    std::vector<Vec2> centered;
    centered.reserve(observations.size());
    for (const Vec2 &point : observations) {
        centered.emplace_back(point - camera.principal);
    }

    std::mt19937 generator(static_cast<std::uint32_t>(random_seed + camera_index * 7919));
    std::uniform_int_distribution<std::size_t> distribution(0, points.size() - 1);
    const std::size_t iterations = points.size() <= 10 ? 800 : 1200;

    *best_score = std::numeric_limits<double>::max();
    int best_inlier_count = 0;
    for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
        std::set<std::size_t> selected;
        while (selected.size() < 4) {
            selected.insert(distribution(generator));
        }

        std::vector<Vec2> sample_points;
        std::vector<Vec3> sample_world;
        sample_points.reserve(4);
        sample_world.reserve(4);
        for (const std::size_t index : selected) {
            sample_points.push_back(centered[index]);
            sample_world.push_back(points[index]);
        }

        std::vector<poselib::CameraPose> poses;
        std::vector<double> focals;
        try {
            poselib::p4pf(sample_points, sample_world, &poses, &focals, true);
        } catch (...) {
            continue;
        }

        for (std::size_t solution = 0; solution < poses.size() && solution < focals.size(); ++solution) {
            const double focal = focals[solution];
            if (!std::isfinite(focal) || focal < camera.min_focal || focal > camera.max_focal) {
                continue;
            }

            double squared_sum = 0.0;
            int inliers = 0;
            int positive = 0;
            for (std::size_t point_index = 0; point_index < points.size(); ++point_index) {
                bool is_positive = false;
                const double squared = ReprojectionSquared(
                    points[point_index],
                    observations[point_index],
                    camera,
                    focal,
                    poses[solution],
                    &is_positive);
                positive += is_positive ? 1 : 0;
                if (squared < 25.0) {
                    ++inliers;
                    squared_sum += squared;
                }
            }
            if (positive < static_cast<int>(points.size() * 0.8) || inliers < 6) {
                continue;
            }

            const double score = std::sqrt(squared_sum / std::max(1, inliers * 2));
            if (inliers > best_inlier_count || (inliers == best_inlier_count && score < *best_score)) {
                best_inlier_count = inliers;
                *best_score = score;
                *best_pose = poses[solution];
                *best_focal = focal;
            }
        }
    }
    return best_inlier_count >= 6 && std::isfinite(*best_score);
}

void ReanchorToCameraZero(Candidate *candidate) {
    const Mat3 rotation0 = candidate->poses[0].R();
    const Vec3 translation0 = candidate->poses[0].t;
    for (Vec3 &point : candidate->points) {
        point = rotation0 * point + translation0;
    }

    PoseArray reanchored{};
    for (int camera_index = 0; camera_index < kCameraCount; ++camera_index) {
        const Mat3 rotation = candidate->poses[camera_index].R() * rotation0.transpose();
        const Vec3 translation = candidate->poses[camera_index].t - rotation * translation0;
        reanchored[camera_index] = poselib::CameraPose(rotation, translation);
    }
    candidate->poses = reanchored;

    const double baseline = candidate->poses[1].center().norm();
    if (!std::isfinite(baseline) || baseline < 1e-8) {
        throw std::runtime_error("Camera 0 and Camera 1 baseline is too small.");
    }
    const double inverse_baseline = 1.0 / baseline;
    for (Vec3 &point : candidate->points) {
        point *= inverse_baseline;
    }
    for (int camera_index = 1; camera_index < kCameraCount; ++camera_index) {
        const Mat3 rotation = candidate->poses[camera_index].R();
        const Vec3 center = candidate->poses[camera_index].center() * inverse_baseline;
        candidate->poses[camera_index].t = -rotation * center;
    }
}

std::vector<Candidate> CreateInitialCandidates(
    const std::vector<PairGeometry> &pairs,
    const std::array<NormalizedCamera, kCameraCount> &cameras,
    const std::array<std::vector<Vec2>, kCameraCount> &observations,
    const int random_seed) {
    std::vector<Candidate> candidates;

    for (const PairGeometry &pair : pairs) {
        const auto focal_pairs = EstimateFocalPairs(pair, cameras);
        for (const auto &[first_focal, second_focal] : focal_pairs) {
            const Mat3 essential = ProjectToEssential(
                CameraMatrix(cameras[pair.second], second_focal).transpose() *
                pair.fundamental *
                CameraMatrix(cameras[pair.first], first_focal));

            std::vector<Vec3> first_bearings;
            std::vector<Vec3> second_bearings;
            first_bearings.reserve(observations[pair.first].size());
            second_bearings.reserve(observations[pair.second].size());
            for (std::size_t point_index = 0; point_index < observations[pair.first].size(); ++point_index) {
                first_bearings.push_back(Bearing(
                    cameras[pair.first], first_focal, observations[pair.first][point_index]));
                second_bearings.push_back(Bearing(
                    cameras[pair.second], second_focal, observations[pair.second][point_index]));
            }

            poselib::CameraPoseVector relative_poses;
            poselib::motion_from_essential(
                essential, first_bearings, second_bearings, &relative_poses);
            for (poselib::CameraPose relative_pose : relative_poses) {
                const double translation_norm = relative_pose.t.norm();
                if (!std::isfinite(translation_norm) || translation_norm < 1e-8) {
                    continue;
                }
                relative_pose.t /= translation_norm;

                Candidate candidate;
                candidate.poses[pair.first] = poselib::CameraPose();
                candidate.poses[pair.second] = relative_pose;
                candidate.focals[pair.first] = first_focal;
                candidate.focals[pair.second] = second_focal;
                candidate.points.reserve(observations[pair.first].size());

                int positive_pair_points = 0;
                for (std::size_t point_index = 0; point_index < observations[pair.first].size(); ++point_index) {
                    const Vec3 point = Triangulate(
                        observations[pair.first][point_index],
                        observations[pair.second][point_index],
                        cameras[pair.first],
                        cameras[pair.second],
                        first_focal,
                        second_focal,
                        relative_pose);
                    candidate.points.push_back(point);
                    if (point.allFinite() && point.z() > 0.0 && relative_pose.apply(point).z() > 0.0) {
                        ++positive_pair_points;
                    }
                }
                if (positive_pair_points < static_cast<int>(candidate.points.size() * 0.8)) {
                    continue;
                }

                const int third = 3 - pair.first - pair.second;
                double third_score = 0.0;
                if (!InitializeThirdCamera(
                        third,
                        cameras[third],
                        observations[third],
                        candidate.points,
                        random_seed,
                        &candidate.poses[third],
                        &candidate.focals[third],
                        &third_score)) {
                    continue;
                }

                try {
                    ReanchorToCameraZero(&candidate);
                } catch (...) {
                    continue;
                }
                int positive_count = 0;
                candidate.score = ScoreCandidate(candidate, cameras, observations, &positive_count);
                if (positive_count == static_cast<int>(candidate.points.size() * kCameraCount) &&
                    std::isfinite(candidate.score)) {
                    candidates.push_back(std::move(candidate));
                }
            }
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate &left, const Candidate &right) {
        return left.score < right.score;
    });
    return candidates;
}

struct ReprojectionCost {
    ReprojectionCost(const Vec2 &observation, const Vec2 &principal)
        : observation_(observation), principal_(principal) {
    }

    template <typename T>
    bool operator()(
        const T *const rotation,
        const T *const center,
        const T *const log_focal,
        const T *const point,
        T *residuals) const {
        T relative[3]{
            point[0] - center[0],
            point[1] - center[1],
            point[2] - center[2],
        };
        T camera_point[3];
        ceres::AngleAxisRotatePoint(rotation, relative, camera_point);
        const T focal = exp(log_focal[0]);
        const T inverse_depth = T(1.0) / camera_point[2];
        const T projected_x = focal * camera_point[0] * inverse_depth + T(principal_.x());
        const T projected_y = focal * camera_point[1] * inverse_depth + T(principal_.y());
        residuals[0] = projected_x - T(observation_.x());
        residuals[1] = projected_y - T(observation_.y());
        return true;
    }

    Vec2 observation_;
    Vec2 principal_;
};

bool OptimizeCandidate(
    Candidate *candidate,
    const std::array<NormalizedCamera, kCameraCount> &cameras,
    const std::array<std::vector<Vec2>, kCameraCount> &observations) {
    std::array<CameraParameters, kCameraCount> parameters{};
    for (int camera_index = 0; camera_index < kCameraCount; ++camera_index) {
        const Mat3 rotation = candidate->poses[camera_index].R();
        ceres::RotationMatrixToAngleAxis(rotation.data(), parameters[camera_index].rotation);
        const Vec3 center = candidate->poses[camera_index].center();
        std::copy(center.data(), center.data() + 3, parameters[camera_index].center);
        parameters[camera_index].log_focal = std::log(candidate->focals[camera_index]);
    }

    std::vector<std::array<double, 3>> points(candidate->points.size());
    for (std::size_t index = 0; index < candidate->points.size(); ++index) {
        std::copy(candidate->points[index].data(), candidate->points[index].data() + 3, points[index].data());
    }

    ceres::Problem problem;
    for (int camera_index = 0; camera_index < kCameraCount; ++camera_index) {
        problem.AddParameterBlock(parameters[camera_index].rotation, 3);
        problem.AddParameterBlock(parameters[camera_index].center, 3);
        problem.AddParameterBlock(&parameters[camera_index].log_focal, 1);
        problem.SetParameterLowerBound(
            &parameters[camera_index].log_focal, 0, std::log(cameras[camera_index].min_focal));
        problem.SetParameterUpperBound(
            &parameters[camera_index].log_focal, 0, std::log(cameras[camera_index].max_focal));
    }

    problem.SetParameterBlockConstant(parameters[0].rotation);
    problem.SetParameterBlockConstant(parameters[0].center);
    problem.SetManifold(parameters[1].center, new ceres::SphereManifold<3>());

    for (std::size_t point_index = 0; point_index < points.size(); ++point_index) {
        for (int camera_index = 0; camera_index < kCameraCount; ++camera_index) {
            auto *cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, 3, 3, 1, 3>(
                new ReprojectionCost(
                    observations[camera_index][point_index], cameras[camera_index].principal));
            problem.AddResidualBlock(
                cost,
                new ceres::HuberLoss(3.0),
                parameters[camera_index].rotation,
                parameters[camera_index].center,
                &parameters[camera_index].log_focal,
                points[point_index].data());
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations = 100;
    options.num_threads = 1;
    options.function_tolerance = 1e-12;
    options.gradient_tolerance = 1e-12;
    options.parameter_tolerance = 1e-10;
    options.minimizer_progress_to_stdout = false;
    options.logging_type = ceres::SILENT;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (!summary.IsSolutionUsable()) {
        return false;
    }

    for (int camera_index = 0; camera_index < kCameraCount; ++camera_index) {
        Mat3 rotation;
        ceres::AngleAxisToRotationMatrix(parameters[camera_index].rotation, rotation.data());
        const Vec3 center(parameters[camera_index].center);
        candidate->poses[camera_index] = poselib::CameraPose(rotation, -rotation * center);
        candidate->focals[camera_index] = std::exp(parameters[camera_index].log_focal);
    }
    for (std::size_t index = 0; index < points.size(); ++index) {
        candidate->points[index] = Vec3(points[index].data());
    }

    int positive_count = 0;
    candidate->rms = ScoreCandidate(*candidate, cameras, observations, &positive_count);
    candidate->score = candidate->rms;
    candidate->median_angle = ComputeMedianTriangulationAngle(candidate->poses, candidate->points);
    candidate->optimized = true;
    return positive_count == static_cast<int>(candidate->points.size() * kCameraCount) &&
           std::isfinite(candidate->rms);
}

std::vector<PairGeometry> EstimatePairGeometry(
    const std::array<std::vector<Vec2>, kCameraCount> &observations,
    const int random_seed,
    bool *planar_degenerate) {
    std::vector<PairGeometry> pairs;
    int planar_pair_count = 0;
    for (int first = 0; first < kCameraCount; ++first) {
        for (int second = first + 1; second < kCameraCount; ++second) {
            poselib::RansacOptions ransac;
            ransac.seed = static_cast<unsigned long>(random_seed + first * 31 + second * 101);
            ransac.min_iterations = 100;
            ransac.max_iterations = 5000;
            ransac.max_epipolar_error = 3.0;
            ransac.max_reproj_error = 3.0;

            poselib::BundleOptions bundle;
            bundle.max_iterations = 50;
            bundle.loss_type = poselib::BundleOptions::LossType::HUBER;
            bundle.loss_scale = 3.0;

            PairGeometry pair;
            pair.first = first;
            pair.second = second;
            const auto fundamental_stats = poselib::estimate_fundamental(
                observations[first],
                observations[second],
                ransac,
                bundle,
                &pair.fundamental,
                &pair.fundamental_inliers);
            Mat3 homography;
            const auto homography_stats = poselib::estimate_homography(
                observations[first],
                observations[second],
                ransac,
                bundle,
                &homography,
                &pair.homography_inliers);

            if (fundamental_stats.num_inliers < 8 || !pair.fundamental.allFinite()) {
                continue;
            }
            pair.fundamental_score = fundamental_stats.model_score;
            if (homography_stats.num_inliers + 1 >= observations[first].size()) {
                ++planar_pair_count;
            }
            pairs.push_back(std::move(pair));
        }
    }
    *planar_degenerate = planar_pair_count >= 2;
    return pairs;
}

std::string HighErrorPointMessage(
    const Candidate &candidate,
    const std::array<NormalizedCamera, kCameraCount> &cameras,
    const std::array<std::vector<Vec2>, kCameraCount> &observations,
    const std::int32_t *point_ids,
    const double threshold) {
    std::vector<int> failed_ids;
    for (std::size_t point_index = 0; point_index < candidate.points.size(); ++point_index) {
        double squared_sum = 0.0;
        for (int camera_index = 0; camera_index < kCameraCount; ++camera_index) {
            squared_sum += ReprojectionSquared(
                candidate.points[point_index],
                observations[camera_index][point_index],
                cameras[camera_index],
                candidate.focals[camera_index],
                candidate.poses[camera_index]);
        }
        const double rms = std::sqrt(squared_sum / (kCameraCount * 2.0));
        if (!std::isfinite(rms) || rms > threshold) {
            failed_ids.push_back(point_ids[point_index]);
        }
    }

    if (failed_ids.empty()) {
        return {};
    }
    std::ostringstream stream;
    stream << "These point IDs have excessive reprojection error: ";
    for (std::size_t index = 0; index < failed_ids.size(); ++index) {
        if (index > 0) {
            stream << ", ";
        }
        stream << failed_ids[index];
    }
    return stream.str();
}

void ApplyKnownScale(
    Candidate *candidate,
    const std::int32_t *point_ids,
    const RT_SolveOptions &options,
    double *applied_scale) {
    int index_a = -1;
    int index_b = -1;
    for (std::size_t index = 0; index < candidate->points.size(); ++index) {
        if (point_ids[index] == options.scale_point_id_a) {
            index_a = static_cast<int>(index);
        }
        if (point_ids[index] == options.scale_point_id_b) {
            index_b = static_cast<int>(index);
        }
    }
    if (index_a < 0 || index_b < 0 || index_a == index_b || options.known_scale_distance <= 0.0) {
        throw std::invalid_argument("Scale point IDs or known scale distance are invalid.");
    }

    const double solved_distance = (candidate->points[index_a] - candidate->points[index_b]).norm();
    if (!std::isfinite(solved_distance) || solved_distance < 1e-8) {
        throw std::runtime_error("The solved scale points are too close to determine scale.");
    }
    *applied_scale = options.known_scale_distance / solved_distance;
    for (Vec3 &point : candidate->points) {
        point *= *applied_scale;
    }
    for (int camera_index = 1; camera_index < kCameraCount; ++camera_index) {
        const Mat3 rotation = candidate->poses[camera_index].R();
        const Vec3 center = candidate->poses[camera_index].center() * *applied_scale;
        candidate->poses[camera_index].t = -rotation * center;
    }
}

void FillOutputs(
    const Candidate &candidate,
    const std::array<NormalizedCamera, kCameraCount> &cameras,
    const std::array<std::vector<Vec2>, kCameraCount> &observations,
    const std::int32_t *point_ids,
    RT_CameraOutput *camera_outputs,
    RT_PointOutput *point_outputs) {
    const Mat3 coordinate_flip = (Mat3() << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0).finished();

    for (int camera_index = 0; camera_index < kCameraCount; ++camera_index) {
        const double focal_pixels = candidate.focals[camera_index] / cameras[camera_index].pixel_scale;
        camera_outputs[camera_index].focal_length_pixels = focal_pixels;
        camera_outputs[camera_index].horizontal_fov_degrees =
            FovFromFocal(cameras[camera_index].width / cameras[camera_index].pixel_scale, focal_pixels);
        camera_outputs[camera_index].vertical_fov_degrees =
            FovFromFocal(cameras[camera_index].height / cameras[camera_index].pixel_scale, focal_pixels);

        const Vec3 center_unity = coordinate_flip * candidate.poses[camera_index].center();
        std::copy(center_unity.data(), center_unity.data() + 3, camera_outputs[camera_index].position);

        const Mat3 camera_to_world_unity =
            coordinate_flip * candidate.poses[camera_index].R().transpose() * coordinate_flip;
        Eigen::Quaterniond rotation(camera_to_world_unity);
        rotation.normalize();
        camera_outputs[camera_index].rotation_xyzw[0] = rotation.x();
        camera_outputs[camera_index].rotation_xyzw[1] = rotation.y();
        camera_outputs[camera_index].rotation_xyzw[2] = rotation.z();
        camera_outputs[camera_index].rotation_xyzw[3] = rotation.w();

        double squared_sum = 0.0;
        for (std::size_t point_index = 0; point_index < candidate.points.size(); ++point_index) {
            squared_sum += ReprojectionSquared(
                candidate.points[point_index],
                observations[camera_index][point_index],
                cameras[camera_index],
                candidate.focals[camera_index],
                candidate.poses[camera_index]);
        }
        camera_outputs[camera_index].reprojection_rms_pixels =
            std::sqrt(squared_sum / std::max<std::size_t>(1, candidate.points.size() * 2)) /
            cameras[camera_index].pixel_scale;
    }

    for (std::size_t point_index = 0; point_index < candidate.points.size(); ++point_index) {
        point_outputs[point_index].id = point_ids[point_index];
        const Vec3 point_unity = coordinate_flip * candidate.points[point_index];
        std::copy(point_unity.data(), point_unity.data() + 3, point_outputs[point_index].position);

        double squared_sum_pixels = 0.0;
        for (int camera_index = 0; camera_index < kCameraCount; ++camera_index) {
            const double squared_normalized = ReprojectionSquared(
                candidate.points[point_index],
                observations[camera_index][point_index],
                cameras[camera_index],
                candidate.focals[camera_index],
                candidate.poses[camera_index]);
            squared_sum_pixels += squared_normalized /
                                  (cameras[camera_index].pixel_scale * cameras[camera_index].pixel_scale);
        }
        point_outputs[point_index].reprojection_rms_pixels =
            std::sqrt(squared_sum_pixels / (kCameraCount * 2.0));
    }
}

} // namespace

const char *RT_CALL RT_GetVersion() {
    return "ReconstructionNative/1.0.0 (PoseLib 2.0.5, Ceres 2.2.0)";
}

std::int32_t RT_CALL RT_SolveThreeView(
    const RT_CameraInput *cameras,
    const std::int32_t *point_ids,
    const RT_Observation *observations,
    const std::int32_t point_count,
    const RT_SolveOptions *options,
    RT_CameraOutput *camera_outputs,
    RT_PointOutput *point_outputs,
    RT_SolveReport *report,
    char *error_buffer,
    const std::int32_t error_buffer_capacity) {
    if (report != nullptr) {
        *report = {};
        report->status = RT_INTERNAL_ERROR;
    }
    WriteError(error_buffer, error_buffer_capacity, "");

    try {
        if (cameras == nullptr || point_ids == nullptr || observations == nullptr || options == nullptr ||
            camera_outputs == nullptr || point_outputs == nullptr || report == nullptr) {
            throw std::invalid_argument("One or more required pointers are null.");
        }
        if (point_count < 8) {
            throw std::invalid_argument("At least 8 shared reference points are required.");
        }
        if (options->known_scale_distance <= 0.0) {
            throw std::invalid_argument("Known scale distance must be greater than zero.");
        }

        std::set<std::int32_t> unique_ids;
        for (int point_index = 0; point_index < point_count; ++point_index) {
            if (!unique_ids.insert(point_ids[point_index]).second) {
                throw std::invalid_argument("Point IDs must be unique.");
            }
        }

        std::array<NormalizedCamera, kCameraCount> normalized_cameras{};
        std::array<std::vector<Vec2>, kCameraCount> normalized_observations;
        for (int camera_index = 0; camera_index < kCameraCount; ++camera_index) {
            const auto &input = cameras[camera_index];
            if (input.width <= 0 || input.height <= 0 ||
                input.min_vertical_fov_degrees <= 1.0 ||
                input.max_vertical_fov_degrees >= 179.0 ||
                input.min_vertical_fov_degrees >= input.max_vertical_fov_degrees) {
                throw std::invalid_argument("Camera dimensions or FOV bounds are invalid.");
            }

            auto &camera = normalized_cameras[camera_index];
            camera.pixel_scale = kNormalizedLongSide / std::max(input.width, input.height);
            camera.width = input.width * camera.pixel_scale;
            camera.height = input.height * camera.pixel_scale;
            camera.principal = Vec2(camera.width * 0.5, camera.height * 0.5);
            camera.min_fov = input.min_vertical_fov_degrees;
            camera.max_fov = input.max_vertical_fov_degrees;
            camera.min_focal = FocalFromVerticalFov(camera.height, camera.max_fov);
            camera.max_focal = FocalFromVerticalFov(camera.height, camera.min_fov);

            auto &camera_observations = normalized_observations[camera_index];
            camera_observations.reserve(point_count);
            for (int point_index = 0; point_index < point_count; ++point_index) {
                const auto &point = observations[camera_index * point_count + point_index];
                if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                    point.x < 0.0 || point.x > input.width || point.y < 0.0 || point.y > input.height) {
                    throw std::invalid_argument("An observation is non-finite or outside its image bounds.");
                }
                camera_observations.emplace_back(
                    point.x * camera.pixel_scale, point.y * camera.pixel_scale);
            }
        }

        bool planar_degenerate = false;
        const auto pairs = EstimatePairGeometry(
            normalized_observations, options->random_seed, &planar_degenerate);
        if (pairs.size() < 2) {
            report->status = RT_DEGENERATE_GEOMETRY;
            WriteError(error_buffer, error_buffer_capacity, "Unable to estimate enough pairwise geometry.");
            return report->status;
        }
        if (planar_degenerate) {
            report->status = RT_DEGENERATE_GEOMETRY;
            WriteError(
                error_buffer,
                error_buffer_capacity,
                "The marked points are dominated by a planar homography. Add points with more depth variation.");
            return report->status;
        }

        auto candidates = CreateInitialCandidates(
            pairs, normalized_cameras, normalized_observations, options->random_seed);
        if (candidates.empty()) {
            report->status = RT_INITIALIZATION_FAILED;
            WriteError(
                error_buffer,
                error_buffer_capacity,
                "No valid three-camera initialization was found. Check FOV bounds, point IDs, and camera baseline.");
            return report->status;
        }

        const int requested_candidates = options->max_candidates > 0 ? options->max_candidates : 12;
        if (candidates.size() > static_cast<std::size_t>(requested_candidates)) {
            candidates.resize(static_cast<std::size_t>(requested_candidates));
        }
        std::vector<Candidate> optimized;
        for (Candidate &candidate : candidates) {
            if (OptimizeCandidate(
                    &candidate, normalized_cameras, normalized_observations)) {
                optimized.push_back(std::move(candidate));
            }
        }
        if (optimized.empty()) {
            report->status = RT_OPTIMIZATION_FAILED;
            WriteError(error_buffer, error_buffer_capacity, "Bundle adjustment did not produce a usable solution.");
            return report->status;
        }

        std::sort(optimized.begin(), optimized.end(), [](const Candidate &left, const Candidate &right) {
            return left.rms < right.rms;
        });
        Candidate best = std::move(optimized.front());

        if (best.median_angle < 1.0) {
            report->status = RT_DEGENERATE_GEOMETRY;
            WriteError(
                error_buffer,
                error_buffer_capacity,
                "Camera parallax is too small; median triangulation angle is below 1 degree.");
            return report->status;
        }

        const double allowed_error =
            options->max_normalized_reprojection_error > 0.0
                ? options->max_normalized_reprojection_error
                : 8.0;
        const std::string point_error = HighErrorPointMessage(
            best,
            normalized_cameras,
            normalized_observations,
            point_ids,
            allowed_error);
        if (!point_error.empty() || best.rms > allowed_error) {
            report->status = RT_HIGH_REPROJECTION_ERROR;
            WriteError(
                error_buffer,
                error_buffer_capacity,
                point_error.empty() ? "The final reprojection error is too high." : point_error);
            return report->status;
        }

        for (int camera_index = 0; camera_index < kCameraCount; ++camera_index) {
            const double vertical_fov =
                FovFromFocal(normalized_cameras[camera_index].height, best.focals[camera_index]);
            if (vertical_fov <= normalized_cameras[camera_index].min_fov + 0.2 ||
                vertical_fov >= normalized_cameras[camera_index].max_fov - 0.2) {
                report->status = RT_INITIALIZATION_FAILED;
                WriteError(
                    error_buffer,
                    error_buffer_capacity,
                    "A solved FOV reached its configured boundary. Expand or correct that camera's FOV range.");
                return report->status;
            }
        }

        if (optimized.size() > 1) {
            const Candidate &second = optimized[1];
            if (second.rms <= best.rms * 1.01 + 1e-6) {
                double maximum_fov_difference = 0.0;
                for (int camera_index = 0; camera_index < kCameraCount; ++camera_index) {
                    const double first_fov =
                        FovFromFocal(normalized_cameras[camera_index].height, best.focals[camera_index]);
                    const double second_fov =
                        FovFromFocal(normalized_cameras[camera_index].height, second.focals[camera_index]);
                    maximum_fov_difference = std::max(maximum_fov_difference, std::abs(first_fov - second_fov));
                }
                if (maximum_fov_difference > 12.0) {
                    report->status = RT_AMBIGUOUS_SOLUTION;
                    WriteError(
                        error_buffer,
                        error_buffer_capacity,
                        "Multiple solutions have similar error but materially different FOV values.");
                    return report->status;
                }
            }
        }

        double applied_scale = 1.0;
        ApplyKnownScale(&best, point_ids, *options, &applied_scale);
        FillOutputs(
            best,
            normalized_cameras,
            normalized_observations,
            point_ids,
            camera_outputs,
            point_outputs);

        report->status = RT_SUCCESS;
        report->point_count = point_count;
        report->inlier_count = point_count;
        report->normalized_reprojection_rms = best.rms;
        report->median_triangulation_angle_degrees = best.median_angle;
        report->applied_scale = applied_scale;
        return RT_SUCCESS;
    } catch (const std::invalid_argument &exception) {
        if (report != nullptr) {
            report->status = RT_INVALID_ARGUMENT;
        }
        WriteError(error_buffer, error_buffer_capacity, exception.what());
        return RT_INVALID_ARGUMENT;
    } catch (const std::exception &exception) {
        if (report != nullptr) {
            report->status = RT_INTERNAL_ERROR;
        }
        WriteError(error_buffer, error_buffer_capacity, exception.what());
        return RT_INTERNAL_ERROR;
    } catch (...) {
        if (report != nullptr) {
            report->status = RT_INTERNAL_ERROR;
        }
        WriteError(error_buffer, error_buffer_capacity, "Unknown native solver error.");
        return RT_INTERNAL_ERROR;
    }
}
