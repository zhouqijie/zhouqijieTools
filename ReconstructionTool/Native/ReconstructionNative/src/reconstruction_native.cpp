#include "reconstruction_native.h"

#include <PoseLib/misc/decompositions.h>
#include <PoseLib/misc/essential.h>
#include <PoseLib/robust.h>
#include <PoseLib/solvers/p4pf.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/sphere_manifold.h>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
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

constexpr int kBaseCameraCount = 3;
constexpr int kMaximumCameraCount = 64;
constexpr double kNormalizedLongSide = 1000.0;
constexpr double kPi = 3.14159265358979323846;

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using PointObservationList = std::vector<std::vector<Vec2>>;
using VisibilityList = std::vector<std::vector<char>>;
using PointConfidenceList = std::vector<std::vector<double>>;

struct NormalizedCamera {
    double width = 0.0;
    double height = 0.0;
    double pixel_scale = 1.0;
    double min_focal = 0.0;
    double max_focal = 0.0;
    double min_fov = 0.0;
    double max_fov = 0.0;
    double confidence = 1.0;
    bool pose_only = false;
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

struct NormalizedLineObservation {
    Vec3 equation = Vec3::Zero();
    double confidence = 1.0;
};

using CameraList = std::vector<NormalizedCamera>;
using LineObservationList = std::vector<std::vector<NormalizedLineObservation>>;

struct Line3D {
    Vec3 point = Vec3::Zero();
    Vec3 direction = Vec3::UnitX();
    double sample_extent = 1.0;
};

struct Candidate {
    std::vector<poselib::CameraPose> poses;
    std::vector<double> focals;
    std::vector<Vec3> points;
    std::vector<Line3D> lines;
    double score = std::numeric_limits<double>::max();
    double rms = std::numeric_limits<double>::max();
    double line_rms = 0.0;
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

std::array<double, 2> LineResiduals(
    const Line3D &line,
    const NormalizedLineObservation &observation,
    const NormalizedCamera &camera,
    const double focal,
    const poselib::CameraPose &pose) {
    std::array<double, 2> residuals{};
    for (int endpoint = 0; endpoint < 2; ++endpoint) {
        const double offset = endpoint == 0 ? -line.sample_extent : line.sample_extent;
        const Vec3 camera_point = pose.apply(line.point + offset * line.direction);
        if (!camera_point.allFinite() || camera_point.z() <= 1e-8) {
            residuals[endpoint] = 1e6;
            continue;
        }
        const double projected_x =
            focal * camera_point.x() / camera_point.z() + camera.principal.x();
        const double projected_y =
            focal * camera_point.y() / camera_point.z() + camera.principal.y();
        residuals[endpoint] =
            observation.equation.dot(Vec3(projected_x, projected_y, 1.0));
    }
    return residuals;
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

double ComputeMedianTriangulationAngle(
    const std::vector<poselib::CameraPose> &poses,
    const std::vector<Vec3> &points,
    const VisibilityList &visibility,
    const CameraList &cameras) {
    std::vector<double> angles;
    angles.reserve(points.size());
    for (std::size_t point_index = 0; point_index < points.size(); ++point_index) {
        double maximum_angle = 0.0;
        for (std::size_t first = 0; first < poses.size(); ++first) {
            if (cameras[first].pose_only || !visibility[first][point_index]) {
                continue;
            }
            for (std::size_t second = first + 1; second < poses.size(); ++second) {
                if (cameras[second].pose_only || !visibility[second][point_index]) {
                    continue;
                }
                const Vec3 ray1 = (points[point_index] - poses[first].center()).normalized();
                const Vec3 ray2 = (points[point_index] - poses[second].center()).normalized();
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
    const CameraList &cameras,
    const PointObservationList &observations,
    const VisibilityList &visibility,
    const PointConfidenceList &point_confidences,
    int *positive_count = nullptr,
    int *visible_count = nullptr) {
    double squared_sum = 0.0;
    double weight_sum = 0.0;
    int positive = 0;
    int visible = 0;
    for (std::size_t point_index = 0; point_index < candidate.points.size(); ++point_index) {
        for (std::size_t camera_index = 0; camera_index < candidate.poses.size(); ++camera_index) {
            if (cameras[camera_index].pose_only ||
                !visibility[camera_index][point_index]) {
                continue;
            }
            visible++;
            bool is_positive = false;
            const double confidence =
                cameras[camera_index].confidence *
                point_confidences[camera_index][point_index];
            squared_sum += confidence *
                           ReprojectionSquared(
                               candidate.points[point_index],
                               observations[camera_index][point_index],
                               cameras[camera_index],
                               candidate.focals[camera_index],
                               candidate.poses[camera_index],
                               &is_positive);
            positive += is_positive ? 1 : 0;
            weight_sum += confidence;
        }
    }

    if (positive_count != nullptr) {
        *positive_count = positive;
    }
    if (visible_count != nullptr) {
        *visible_count = visible;
    }
    return std::sqrt(squared_sum / std::max(1e-8, weight_sum * 2.0));
}

double ScoreLines(
    const Candidate &candidate,
    const CameraList &cameras,
    const LineObservationList &observations,
    const VisibilityList &visibility,
    double *weighted_squared_sum = nullptr,
    double *weight_sum = nullptr) {
    double squared_sum = 0.0;
    double weights = 0.0;
    for (std::size_t line_index = 0; line_index < candidate.lines.size(); ++line_index) {
        for (std::size_t camera_index = 0; camera_index < candidate.poses.size(); ++camera_index) {
            if (cameras[camera_index].pose_only ||
                !visibility[camera_index][line_index]) {
                continue;
            }
            const NormalizedLineObservation &observation =
                observations[camera_index][line_index];
            const double confidence =
                cameras[camera_index].confidence * observation.confidence;
            const auto residuals = LineResiduals(
                candidate.lines[line_index],
                observation,
                cameras[camera_index],
                candidate.focals[camera_index],
                candidate.poses[camera_index]);
            squared_sum += confidence *
                           (residuals[0] * residuals[0] + residuals[1] * residuals[1]);
            weights += confidence;
        }
    }
    if (weighted_squared_sum != nullptr) {
        *weighted_squared_sum = squared_sum;
    }
    if (weight_sum != nullptr) {
        *weight_sum = weights;
    }
    return candidate.lines.empty()
               ? 0.0
               : std::sqrt(squared_sum / std::max(1e-8, weights * 2.0));
}

double CombinedCandidateScore(
    const Candidate &candidate,
    const CameraList &cameras,
    const VisibilityList &point_visibility,
    const PointConfidenceList &point_confidences,
    const LineObservationList &line_observations,
    const VisibilityList &line_visibility) {
    double point_weight = 0.0;
    for (std::size_t camera_index = 0; camera_index < candidate.poses.size(); ++camera_index) {
        if (cameras[camera_index].pose_only) {
            continue;
        }
        for (std::size_t point_index = 0; point_index < candidate.points.size(); ++point_index) {
            if (point_visibility[camera_index][point_index]) {
                point_weight +=
                    cameras[camera_index].confidence *
                    point_confidences[camera_index][point_index];
            }
        }
    }
    double line_squared_sum = 0.0;
    double line_weight = 0.0;
    ScoreLines(
        candidate,
        cameras,
        line_observations,
        line_visibility,
        &line_squared_sum,
        &line_weight);
    const double point_squared_sum = candidate.rms * candidate.rms * point_weight * 2.0;
    return std::sqrt(
        (point_squared_sum + line_squared_sum) /
        std::max(1e-8, (point_weight + line_weight) * 2.0));
}

bool InitializeCandidateLines(
    Candidate *candidate,
    const CameraList &cameras,
    const LineObservationList &observations,
    const VisibilityList &visibility) {
    candidate->lines.clear();
    if (observations[0].empty()) {
        return true;
    }

    Vec3 anchor = Vec3::Zero();
    for (const Vec3 &point : candidate->points) {
        anchor += point;
    }
    anchor /= static_cast<double>(candidate->points.size());

    double cloud_extent = 0.0;
    for (const Vec3 &point : candidate->points) {
        cloud_extent = std::max(cloud_extent, (point - anchor).norm());
    }
    const double sample_extent = std::max(0.25, cloud_extent * 0.5);

    candidate->lines.reserve(observations[0].size());
    for (std::size_t line_index = 0; line_index < observations[0].size(); ++line_index) {
        Mat3 normal_covariance = Mat3::Zero();
        Vec3 plane_rhs = Vec3::Zero();
        double total_weight = 0.0;

        int observed_camera_count = 0;
        for (std::size_t camera_index = 0; camera_index < candidate->poses.size(); ++camera_index) {
            if (cameras[camera_index].pose_only ||
                !visibility[camera_index][line_index]) {
                continue;
            }
            observed_camera_count++;
            const Vec3 equation = observations[camera_index][line_index].equation;
            Vec3 camera_normal(
                equation.x() * candidate->focals[camera_index],
                equation.y() * candidate->focals[camera_index],
                equation.x() * cameras[camera_index].principal.x() +
                    equation.y() * cameras[camera_index].principal.y() +
                    equation.z());
            if (!camera_normal.allFinite() || camera_normal.norm() < 1e-10) {
                return false;
            }
            camera_normal.normalize();
            const Vec3 world_normal =
                (candidate->poses[camera_index].R().transpose() * camera_normal).normalized();
            const double weight =
                cameras[camera_index].confidence *
                observations[camera_index][line_index].confidence;
            const double plane_offset =
                world_normal.dot(candidate->poses[camera_index].center());
            normal_covariance += weight * world_normal * world_normal.transpose();
            plane_rhs += weight * world_normal * plane_offset;
            total_weight += weight;
        }

        Eigen::SelfAdjointEigenSolver<Mat3> eigen_solver(normal_covariance);
        if (observed_camera_count < 2 ||
            eigen_solver.info() != Eigen::Success ||
            eigen_solver.eigenvalues().y() < total_weight * 1e-7) {
            return false;
        }

        Line3D line;
        line.direction = eigen_solver.eigenvectors().col(0).normalized();
        const double gauge_weight = std::max(1e-6, total_weight);
        const Mat3 system =
            normal_covariance + gauge_weight * line.direction * line.direction.transpose();
        const Vec3 right_hand_side =
            plane_rhs + gauge_weight * line.direction * line.direction.dot(anchor);
        line.point = system.ldlt().solve(right_hand_side);
        line.sample_extent = sample_extent;
        if (!line.point.allFinite() || !line.direction.allFinite()) {
            return false;
        }
        candidate->lines.push_back(line);
    }
    return true;
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
    const CameraList &cameras) {
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
    const int required_inliers = std::max(
        4,
        std::min(
            6,
            static_cast<int>(
                std::ceil(static_cast<double>(points.size()) * 0.7))));
    const int required_positive = points.size() < 6
        ? static_cast<int>(points.size())
        : static_cast<int>(points.size() * 0.8);

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
                if (squared < 64.0) {
                    ++inliers;
                    squared_sum += squared;
                }
            }
            if (positive < required_positive || inliers < required_inliers) {
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
    return best_inlier_count >= required_inliers && std::isfinite(*best_score);
}

void ReanchorToCameraZero(Candidate *candidate) {
    const Mat3 rotation0 = candidate->poses[0].R();
    const Vec3 translation0 = candidate->poses[0].t;
    for (Vec3 &point : candidate->points) {
        point = rotation0 * point + translation0;
    }

    std::vector<poselib::CameraPose> reanchored(candidate->poses.size());
    for (std::size_t camera_index = 0; camera_index < candidate->poses.size(); ++camera_index) {
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
    for (std::size_t camera_index = 1; camera_index < candidate->poses.size(); ++camera_index) {
        const Mat3 rotation = candidate->poses[camera_index].R();
        const Vec3 center = candidate->poses[camera_index].center() * inverse_baseline;
        candidate->poses[camera_index].t = -rotation * center;
    }
}

std::vector<Candidate> CreateInitialCandidates(
    const std::vector<PairGeometry> &pairs,
    const CameraList &cameras,
    const PointObservationList &observations,
    const VisibilityList &visibility,
    const PointConfidenceList &point_confidences,
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
                candidate.poses.resize(kBaseCameraCount);
                candidate.focals.resize(kBaseCameraCount);
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
                int visible_count = 0;
                candidate.score = ScoreCandidate(
                    candidate,
                    cameras,
                    observations,
                    visibility,
                    point_confidences,
                    &positive_count,
                    &visible_count);
                if (positive_count == visible_count &&
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

// 使用固定三维点线细化单个附加相机。
bool OptimizeFixedStructureCamera(
    Candidate *candidate,
    const CameraList &cameras,
    const PointObservationList &observations,
    const VisibilityList &point_visibility,
    const PointConfidenceList &point_confidences,
    const LineObservationList &line_observations,
    const VisibilityList &line_visibility,
    std::size_t camera_index);

// 使用各附加机位实际可见的基础点初始化其位姿和焦距。
bool InitializeAdditionalCameras(
    Candidate *candidate,
    const CameraList &cameras,
    const PointObservationList &observations,
    const VisibilityList &visibility,
    const PointConfidenceList &point_confidences,
    const LineObservationList &line_observations,
    const VisibilityList &line_visibility,
    const int random_seed,
    int *failed_camera_index) {
    for (std::size_t camera_index = kBaseCameraCount;
         camera_index < cameras.size();
         ++camera_index) {
        std::vector<Vec2> visible_observations;
        std::vector<Vec3> visible_points;
        for (std::size_t point_index = 0; point_index < candidate->points.size(); ++point_index) {
            if (visibility[camera_index][point_index]) {
                visible_observations.push_back(observations[camera_index][point_index]);
                visible_points.push_back(candidate->points[point_index]);
            }
        }

        poselib::CameraPose pose;
        double focal = 0.0;
        double score = 0.0;
        if (!InitializeThirdCamera(
                static_cast<int>(camera_index),
                cameras[camera_index],
                visible_observations,
                visible_points,
                random_seed,
                &pose,
                &focal,
                &score)) {
            *failed_camera_index = static_cast<int>(camera_index);
            return false;
        }
        candidate->poses.push_back(pose);
        candidate->focals.push_back(focal);
        if (!OptimizeFixedStructureCamera(
                candidate,
                cameras,
                observations,
                visibility,
                point_confidences,
                line_observations,
                line_visibility,
                camera_index)) {
            *failed_camera_index = static_cast<int>(camera_index);
            return false;
        }
    }
    return true;
}

// 使用两个以上参与公共重建的附加机位共同初始化新点。
bool InitializeAdditionalPoints(
    Candidate *candidate,
    const CameraList &cameras,
    const PointObservationList &observations,
    const VisibilityList &visibility,
    const int base_point_count,
    int *failed_point_index) {
    const std::size_t point_count = observations[0].size();
    for (std::size_t point_index = static_cast<std::size_t>(base_point_count);
         point_index < point_count;
         ++point_index) {
        int best_first = -1;
        int best_second = -1;
        double best_angle = 0.0;

        // 选择观察射线夹角最大的相机对进行初始三角化。
        for (std::size_t first = kBaseCameraCount; first < cameras.size(); ++first) {
            if (cameras[first].pose_only || !visibility[first][point_index]) {
                continue;
            }
            const Vec3 first_ray =
                candidate->poses[first].R().transpose() *
                Bearing(
                    cameras[first],
                    candidate->focals[first],
                    observations[first][point_index]);
            for (std::size_t second = first + 1; second < cameras.size(); ++second) {
                if (cameras[second].pose_only || !visibility[second][point_index]) {
                    continue;
                }
                const Vec3 second_ray =
                    candidate->poses[second].R().transpose() *
                    Bearing(
                        cameras[second],
                        candidate->focals[second],
                        observations[second][point_index]);
                const double cosine =
                    std::clamp(first_ray.dot(second_ray), -1.0, 1.0);
                const double angle = RadiansToDegrees(std::acos(cosine));
                if (angle > best_angle) {
                    best_first = static_cast<int>(first);
                    best_second = static_cast<int>(second);
                    best_angle = angle;
                }
            }
        }
        if (best_first < 0 || best_second < 0 || best_angle < 0.25) {
            *failed_point_index = static_cast<int>(point_index);
            return false;
        }

        const poselib::CameraPose &first_pose = candidate->poses[best_first];
        const poselib::CameraPose &second_pose = candidate->poses[best_second];
        const Mat3 relative_rotation =
            second_pose.R() * first_pose.R().transpose();
        const Vec3 relative_translation =
            second_pose.t - relative_rotation * first_pose.t;
        const poselib::CameraPose second_from_first(
            relative_rotation,
            relative_translation);
        const Vec3 point_in_first = Triangulate(
            observations[best_first][point_index],
            observations[best_second][point_index],
            cameras[best_first],
            cameras[best_second],
            candidate->focals[best_first],
            candidate->focals[best_second],
            second_from_first);
        const Vec3 point =
            first_pose.R().transpose() * (point_in_first - first_pose.t);
        if (!point.allFinite()) {
            *failed_point_index = static_cast<int>(point_index);
            return false;
        }

        for (std::size_t camera_index = kBaseCameraCount;
             camera_index < cameras.size();
             ++camera_index) {
            if (!cameras[camera_index].pose_only &&
                visibility[camera_index][point_index] &&
                candidate->poses[camera_index].apply(point).z() <= 1e-8) {
                *failed_point_index = static_cast<int>(point_index);
                return false;
            }
        }
        candidate->points.push_back(point);
    }
    return candidate->points.size() == point_count;
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

struct LineReprojectionCost {
    LineReprojectionCost(
        const Vec3 &equation,
        const Vec2 &principal,
        const double sample_extent)
        : equation_(equation), principal_(principal), sample_extent_(sample_extent) {
    }

    template <typename T>
    bool operator()(
        const T *const rotation,
        const T *const center,
        const T *const log_focal,
        const T *const line_point,
        const T *const line_direction,
        T *residuals) const {
        const T focal = exp(log_focal[0]);
        for (int endpoint = 0; endpoint < 2; ++endpoint) {
            const T offset = T(endpoint == 0 ? -sample_extent_ : sample_extent_);
            T relative[3]{
                line_point[0] + offset * line_direction[0] - center[0],
                line_point[1] + offset * line_direction[1] - center[1],
                line_point[2] + offset * line_direction[2] - center[2],
            };
            T camera_point[3];
            ceres::AngleAxisRotatePoint(rotation, relative, camera_point);
            const T inverse_depth = T(1.0) / camera_point[2];
            const T projected_x =
                focal * camera_point[0] * inverse_depth + T(principal_.x());
            const T projected_y =
                focal * camera_point[1] * inverse_depth + T(principal_.y());
            residuals[endpoint] =
                T(equation_.x()) * projected_x +
                T(equation_.y()) * projected_y +
                T(equation_.z());
        }
        return true;
    }

    Vec3 equation_;
    Vec2 principal_;
    double sample_extent_;
};

struct LineGaugeCost {
    LineGaugeCost(const Vec3 &anchor, const double inverse_extent)
        : anchor_(anchor), inverse_extent_(inverse_extent) {
    }

    template <typename T>
    bool operator()(
        const T *const line_point,
        const T *const line_direction,
        T *residuals) const {
        residuals[0] =
            T(inverse_extent_) *
            ((line_point[0] - T(anchor_.x())) * line_direction[0] +
             (line_point[1] - T(anchor_.y())) * line_direction[1] +
             (line_point[2] - T(anchor_.z())) * line_direction[2]);
        return true;
    }

    Vec3 anchor_;
    double inverse_extent_;
};

bool OptimizeCandidate(
    Candidate *candidate,
    const CameraList &cameras,
    const PointObservationList &observations,
    const VisibilityList &point_visibility,
    const PointConfidenceList &point_confidences,
    const LineObservationList &line_observations,
    const VisibilityList &line_visibility) {
    std::vector<CameraParameters> parameters(candidate->poses.size());
    for (std::size_t camera_index = 0; camera_index < candidate->poses.size(); ++camera_index) {
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

    std::vector<std::array<double, 3>> line_points(candidate->lines.size());
    std::vector<std::array<double, 3>> line_directions(candidate->lines.size());
    for (std::size_t index = 0; index < candidate->lines.size(); ++index) {
        std::copy(
            candidate->lines[index].point.data(),
            candidate->lines[index].point.data() + 3,
            line_points[index].data());
        std::copy(
            candidate->lines[index].direction.data(),
            candidate->lines[index].direction.data() + 3,
            line_directions[index].data());
    }

    ceres::Problem problem;
    for (std::size_t camera_index = 0; camera_index < candidate->poses.size(); ++camera_index) {
        problem.AddParameterBlock(parameters[camera_index].rotation, 3);
        problem.AddParameterBlock(parameters[camera_index].center, 3);
        problem.AddParameterBlock(&parameters[camera_index].log_focal, 1);
        problem.SetParameterLowerBound(
            &parameters[camera_index].log_focal, 0, std::log(cameras[camera_index].min_focal));
        problem.SetParameterUpperBound(
            &parameters[camera_index].log_focal, 0, std::log(cameras[camera_index].max_focal));
        if (cameras[camera_index].pose_only) {
            problem.SetParameterBlockConstant(parameters[camera_index].rotation);
            problem.SetParameterBlockConstant(parameters[camera_index].center);
            problem.SetParameterBlockConstant(&parameters[camera_index].log_focal);
        }
    }

    problem.SetParameterBlockConstant(parameters[0].rotation);
    problem.SetParameterBlockConstant(parameters[0].center);
    problem.SetManifold(parameters[1].center, new ceres::SphereManifold<3>());

    for (std::size_t point_index = 0; point_index < points.size(); ++point_index) {
        for (std::size_t camera_index = 0; camera_index < candidate->poses.size(); ++camera_index) {
            if (cameras[camera_index].pose_only ||
                !point_visibility[camera_index][point_index]) {
                continue;
            }
            auto *cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, 3, 3, 1, 3>(
                new ReprojectionCost(
                    observations[camera_index][point_index], cameras[camera_index].principal));
            problem.AddResidualBlock(
                cost,
                new ceres::ScaledLoss(
                    new ceres::HuberLoss(3.0),
                    cameras[camera_index].confidence *
                        point_confidences[camera_index][point_index],
                    ceres::TAKE_OWNERSHIP),
                parameters[camera_index].rotation,
                parameters[camera_index].center,
                &parameters[camera_index].log_focal,
                points[point_index].data());
        }
    }

    Vec3 line_anchor = Vec3::Zero();
    for (const Vec3 &point : candidate->points) {
        line_anchor += point;
    }
    line_anchor /= static_cast<double>(candidate->points.size());
    for (std::size_t line_index = 0; line_index < candidate->lines.size(); ++line_index) {
        problem.AddParameterBlock(line_points[line_index].data(), 3);
        problem.AddParameterBlock(line_directions[line_index].data(), 3);
        problem.SetManifold(line_directions[line_index].data(), new ceres::SphereManifold<3>());

        for (std::size_t camera_index = 0; camera_index < candidate->poses.size(); ++camera_index) {
            if (cameras[camera_index].pose_only ||
                !line_visibility[camera_index][line_index]) {
                continue;
            }
            const NormalizedLineObservation &observation =
                line_observations[camera_index][line_index];
            auto *cost =
                new ceres::AutoDiffCostFunction<LineReprojectionCost, 2, 3, 3, 1, 3, 3>(
                    new LineReprojectionCost(
                        observation.equation,
                        cameras[camera_index].principal,
                        candidate->lines[line_index].sample_extent));
            problem.AddResidualBlock(
                cost,
                new ceres::ScaledLoss(
                    new ceres::HuberLoss(3.0),
                    cameras[camera_index].confidence * observation.confidence,
                    ceres::TAKE_OWNERSHIP),
                parameters[camera_index].rotation,
                parameters[camera_index].center,
                &parameters[camera_index].log_focal,
                line_points[line_index].data(),
                line_directions[line_index].data());
        }

        auto *gauge_cost = new ceres::AutoDiffCostFunction<LineGaugeCost, 1, 3, 3>(
            new LineGaugeCost(
                line_anchor,
                1.0 / std::max(1e-6, candidate->lines[line_index].sample_extent)));
        problem.AddResidualBlock(
            gauge_cost,
            nullptr,
            line_points[line_index].data(),
            line_directions[line_index].data());
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

    for (std::size_t camera_index = 0; camera_index < candidate->poses.size(); ++camera_index) {
        Mat3 rotation;
        ceres::AngleAxisToRotationMatrix(parameters[camera_index].rotation, rotation.data());
        const Vec3 center(parameters[camera_index].center);
        candidate->poses[camera_index] = poselib::CameraPose(rotation, -rotation * center);
        candidate->focals[camera_index] = std::exp(parameters[camera_index].log_focal);
    }
    for (std::size_t index = 0; index < points.size(); ++index) {
        candidate->points[index] = Vec3(points[index].data());
    }
    for (std::size_t index = 0; index < candidate->lines.size(); ++index) {
        candidate->lines[index].direction = Vec3(line_directions[index].data()).normalized();
        candidate->lines[index].point = Vec3(line_points[index].data());
        candidate->lines[index].point -=
            candidate->lines[index].direction *
            candidate->lines[index].direction.dot(candidate->lines[index].point - line_anchor);
    }

    int positive_count = 0;
    int visible_count = 0;
    candidate->rms = ScoreCandidate(
        *candidate,
        cameras,
        observations,
        point_visibility,
        point_confidences,
        &positive_count,
        &visible_count);
    candidate->line_rms = ScoreLines(
        *candidate,
        cameras,
        line_observations,
        line_visibility);
    candidate->score = CombinedCandidateScore(
        *candidate,
        cameras,
        point_visibility,
        point_confidences,
        line_observations,
        line_visibility);
    candidate->median_angle = ComputeMedianTriangulationAngle(
        candidate->poses,
        candidate->points,
        point_visibility,
        cameras);
    candidate->optimized = true;
    return positive_count == visible_count &&
           std::isfinite(candidate->rms) &&
           std::isfinite(candidate->line_rms) &&
           std::isfinite(candidate->score);
}

// 使用固定三维点线细化单个附加相机。
bool OptimizeFixedStructureCamera(
    Candidate *candidate,
    const CameraList &cameras,
    const PointObservationList &observations,
    const VisibilityList &point_visibility,
    const PointConfidenceList &point_confidences,
    const LineObservationList &line_observations,
    const VisibilityList &line_visibility,
    const std::size_t camera_index) {
    CameraParameters parameters;
    const Mat3 rotation = candidate->poses[camera_index].R();
    ceres::RotationMatrixToAngleAxis(rotation.data(), parameters.rotation);
    const Vec3 center = candidate->poses[camera_index].center();
    std::copy(center.data(), center.data() + 3, parameters.center);
    parameters.log_focal = std::log(candidate->focals[camera_index]);

    ceres::Problem problem;
    problem.AddParameterBlock(parameters.rotation, 3);
    problem.AddParameterBlock(parameters.center, 3);
    problem.AddParameterBlock(&parameters.log_focal, 1);
    problem.SetParameterLowerBound(
        &parameters.log_focal, 0, std::log(cameras[camera_index].min_focal));
    problem.SetParameterUpperBound(
        &parameters.log_focal, 0, std::log(cameras[camera_index].max_focal));

    // 点和线参数全部固定，只有当前相机参数允许变化。
    for (std::size_t point_index = 0;
         point_index < candidate->points.size();
         ++point_index) {
        if (!point_visibility[camera_index][point_index]) {
            continue;
        }
        double *point = candidate->points[point_index].data();
        problem.AddParameterBlock(point, 3);
        problem.SetParameterBlockConstant(point);
        auto *cost =
            new ceres::AutoDiffCostFunction<ReprojectionCost, 2, 3, 3, 1, 3>(
                new ReprojectionCost(
                    observations[camera_index][point_index],
                    cameras[camera_index].principal));
        problem.AddResidualBlock(
            cost,
            new ceres::ScaledLoss(
                new ceres::HuberLoss(3.0),
                cameras[camera_index].confidence *
                    point_confidences[camera_index][point_index],
                ceres::TAKE_OWNERSHIP),
            parameters.rotation,
            parameters.center,
            &parameters.log_focal,
            point);
    }

    for (std::size_t line_index = 0;
         line_index < candidate->lines.size();
         ++line_index) {
        if (!line_visibility[camera_index][line_index]) {
            continue;
        }
        Line3D &line = candidate->lines[line_index];
        problem.AddParameterBlock(line.point.data(), 3);
        problem.AddParameterBlock(line.direction.data(), 3);
        problem.SetParameterBlockConstant(line.point.data());
        problem.SetParameterBlockConstant(line.direction.data());
        const NormalizedLineObservation &observation =
            line_observations[camera_index][line_index];
        auto *cost =
            new ceres::AutoDiffCostFunction<
                LineReprojectionCost,
                2,
                3,
                3,
                1,
                3,
                3>(
                new LineReprojectionCost(
                    observation.equation,
                    cameras[camera_index].principal,
                    line.sample_extent));
        problem.AddResidualBlock(
            cost,
            new ceres::ScaledLoss(
                new ceres::HuberLoss(3.0),
                cameras[camera_index].confidence * observation.confidence,
                ceres::TAKE_OWNERSHIP),
            parameters.rotation,
            parameters.center,
            &parameters.log_focal,
            line.point.data(),
            line.direction.data());
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 75;
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

    Mat3 optimized_rotation;
    ceres::AngleAxisToRotationMatrix(
        parameters.rotation,
        optimized_rotation.data());
    const Vec3 optimized_center(parameters.center);
    candidate->poses[camera_index] =
        poselib::CameraPose(
            optimized_rotation,
            -optimized_rotation * optimized_center);
    candidate->focals[camera_index] = std::exp(parameters.log_focal);

    for (std::size_t point_index = 0;
         point_index < candidate->points.size();
         ++point_index) {
        if (point_visibility[camera_index][point_index] &&
            candidate->poses[camera_index]
                    .apply(candidate->points[point_index])
                    .z() <= 1e-8) {
            return false;
        }
    }
    return true;
}

// 固定公共三维结构，分别优化仅定位机位的位置、旋转和焦距。
bool OptimizePoseOnlyCameras(
    Candidate *candidate,
    const CameraList &cameras,
    const PointObservationList &observations,
    const VisibilityList &point_visibility,
    const PointConfidenceList &point_confidences,
    const LineObservationList &line_observations,
    const VisibilityList &line_visibility,
    int *failed_camera_index) {
    for (std::size_t camera_index = kBaseCameraCount;
         camera_index < candidate->poses.size();
         ++camera_index) {
        if (!cameras[camera_index].pose_only) {
            continue;
        }
        if (!OptimizeFixedStructureCamera(
                candidate,
                cameras,
                observations,
                point_visibility,
                point_confidences,
                line_observations,
                line_visibility,
                camera_index)) {
            *failed_camera_index = static_cast<int>(camera_index);
            return false;
        }
    }
    return true;
}

std::vector<PairGeometry> EstimatePairGeometry(
    const PointObservationList &observations,
    const int random_seed,
    bool *planar_degenerate) {
    std::vector<PairGeometry> pairs;
    int planar_pair_count = 0;
    for (int first = 0; first < kBaseCameraCount; ++first) {
        for (int second = first + 1; second < kBaseCameraCount; ++second) {
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
    const CameraList &cameras,
    const PointObservationList &observations,
    const VisibilityList &visibility,
    const PointConfidenceList &point_confidences,
    const std::int32_t *point_ids,
    const double threshold) {
    std::vector<int> failed_ids;
    for (std::size_t point_index = 0; point_index < candidate.points.size(); ++point_index) {
        double squared_sum = 0.0;
        double weight_sum = 0.0;
        for (std::size_t camera_index = 0; camera_index < candidate.poses.size(); ++camera_index) {
            if (cameras[camera_index].pose_only ||
                !visibility[camera_index][point_index]) {
                continue;
            }
            const double confidence =
                cameras[camera_index].confidence *
                point_confidences[camera_index][point_index];
            squared_sum += confidence *
                           ReprojectionSquared(
                               candidate.points[point_index],
                               observations[camera_index][point_index],
                               cameras[camera_index],
                               candidate.focals[camera_index],
                               candidate.poses[camera_index]);
            weight_sum += confidence;
        }
        const double rms = std::sqrt(squared_sum / std::max(1e-8, weight_sum * 2.0));
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

std::string HighErrorLineMessage(
    const Candidate &candidate,
    const CameraList &cameras,
    const LineObservationList &observations,
    const VisibilityList &visibility,
    const std::int32_t *line_ids,
    const double threshold) {
    std::ostringstream stream;
    bool has_failure = false;
    for (std::size_t line_index = 0; line_index < candidate.lines.size(); ++line_index) {
        double squared_sum = 0.0;
        double weight_sum = 0.0;
        for (std::size_t camera_index = 0; camera_index < candidate.poses.size(); ++camera_index) {
            if (cameras[camera_index].pose_only ||
                !visibility[camera_index][line_index]) {
                continue;
            }
            const NormalizedLineObservation &observation =
                observations[camera_index][line_index];
            const double confidence =
                cameras[camera_index].confidence * observation.confidence;
            const auto residuals = LineResiduals(
                candidate.lines[line_index],
                observation,
                cameras[camera_index],
                candidate.focals[camera_index],
                candidate.poses[camera_index]);
            squared_sum += confidence *
                           (residuals[0] * residuals[0] + residuals[1] * residuals[1]);
            weight_sum += confidence;
        }
        const double rms = std::sqrt(squared_sum / std::max(1e-8, weight_sum * 2.0));
        if (std::isfinite(rms) && rms <= threshold) {
            continue;
        }

        if (!has_failure) {
            stream << "These line IDs have excessive reprojection error:\n";
            has_failure = true;
        }

        bool has_camera_detail = false;
        for (std::size_t camera_index = 0;
             camera_index < candidate.poses.size();
             ++camera_index) {
            if (cameras[camera_index].pose_only ||
                !visibility[camera_index][line_index]) {
                continue;
            }
            const NormalizedLineObservation &observation =
                observations[camera_index][line_index];
            const auto residuals = LineResiduals(
                candidate.lines[line_index],
                observation,
                cameras[camera_index],
                candidate.focals[camera_index],
                candidate.poses[camera_index]);
            const double camera_rms = std::sqrt(
                (residuals[0] * residuals[0] + residuals[1] * residuals[1]) * 0.5);
            if (std::isfinite(camera_rms) && camera_rms <= threshold) {
                continue;
            }

            const double pixel_scale = cameras[camera_index].pixel_scale;
            stream << "Camera " << camera_index
                   << " / Line " << line_ids[line_index]
                   << ": RMS=" << std::fixed << std::setprecision(2) << camera_rms
                   << " normalized px (" << camera_rms / pixel_scale
                   << " source-image px), allowed=" << threshold
                   << " normalized px (" << threshold / pixel_scale
                   << " source-image px), confidence="
                   << cameras[camera_index].confidence * observation.confidence
                   << ".\n";
            has_camera_detail = true;
        }
        if (!has_camera_detail) {
            stream << "Line " << line_ids[line_index]
                   << ": combined RMS=" << std::fixed << std::setprecision(2) << rms
                   << " normalized px, allowed=" << threshold
                   << " normalized px.\n";
        }
    }

    if (!has_failure) {
        return {};
    }
    stream << "Parallel lines are supported. Adjust the listed Canvas RefLine2D; "
              "the projected 3D line is too far from that marked 2D line.";
    return stream.str();
}

// 汇总仅定位机位自身的点线重投影误差。
std::string HighErrorPoseOnlyCameraMessage(
    const Candidate &candidate,
    const CameraList &cameras,
    const PointObservationList &point_observations,
    const VisibilityList &point_visibility,
    const PointConfidenceList &point_confidences,
    const LineObservationList &line_observations,
    const VisibilityList &line_visibility,
    const std::int32_t *point_ids,
    const std::int32_t *line_ids,
    const double threshold) {
    std::ostringstream stream;
    bool has_failure = false;
    for (std::size_t camera_index = kBaseCameraCount;
         camera_index < candidate.poses.size();
         ++camera_index) {
        if (!cameras[camera_index].pose_only) {
            continue;
        }

        double squared_sum = 0.0;
        double weight_sum = 0.0;
        for (std::size_t point_index = 0;
             point_index < candidate.points.size();
             ++point_index) {
            if (!point_visibility[camera_index][point_index]) {
                continue;
            }
            const double confidence =
                cameras[camera_index].confidence *
                point_confidences[camera_index][point_index];
            squared_sum += confidence *
                           ReprojectionSquared(
                               candidate.points[point_index],
                               point_observations[camera_index][point_index],
                               cameras[camera_index],
                               candidate.focals[camera_index],
                               candidate.poses[camera_index]);
            weight_sum += confidence;
        }
        for (std::size_t line_index = 0;
             line_index < candidate.lines.size();
             ++line_index) {
            if (!line_visibility[camera_index][line_index]) {
                continue;
            }
            const NormalizedLineObservation &observation =
                line_observations[camera_index][line_index];
            const double confidence =
                cameras[camera_index].confidence * observation.confidence;
            const auto residuals = LineResiduals(
                candidate.lines[line_index],
                observation,
                cameras[camera_index],
                candidate.focals[camera_index],
                candidate.poses[camera_index]);
            squared_sum += confidence *
                           (residuals[0] * residuals[0] +
                            residuals[1] * residuals[1]);
            weight_sum += confidence;
        }

        const double rms =
            std::sqrt(squared_sum / std::max(1e-8, weight_sum * 2.0));
        if (std::isfinite(rms) && rms <= threshold) {
            continue;
        }

        if (!has_failure) {
            stream << "These pose-only cameras have excessive reprojection error:\n";
            has_failure = true;
        }

        stream << "Camera " << camera_index
               << ": combined RMS=" << std::fixed << std::setprecision(2) << rms
               << " normalized px, allowed=" << threshold << " normalized px.\n";
        for (std::size_t point_index = 0;
             point_index < candidate.points.size();
             ++point_index) {
            if (!point_visibility[camera_index][point_index]) {
                continue;
            }
            const double point_rms = std::sqrt(
                ReprojectionSquared(
                    candidate.points[point_index],
                    point_observations[camera_index][point_index],
                    cameras[camera_index],
                    candidate.focals[camera_index],
                    candidate.poses[camera_index]) *
                0.5);
            if (std::isfinite(point_rms) && point_rms <= threshold) {
                continue;
            }
            const double pixel_scale = cameras[camera_index].pixel_scale;
            stream << "Camera " << camera_index
                   << " / Point " << point_ids[point_index]
                   << ": RMS=" << point_rms
                   << " normalized px (" << point_rms / pixel_scale
                   << " source-image px), allowed=" << threshold
                   << " normalized px (" << threshold / pixel_scale
                   << " source-image px).\n";
        }
        for (std::size_t line_index = 0;
             line_index < candidate.lines.size();
             ++line_index) {
            if (!line_visibility[camera_index][line_index]) {
                continue;
            }
            const NormalizedLineObservation &observation =
                line_observations[camera_index][line_index];
            const auto residuals = LineResiduals(
                candidate.lines[line_index],
                observation,
                cameras[camera_index],
                candidate.focals[camera_index],
                candidate.poses[camera_index]);
            const double line_rms = std::sqrt(
                (residuals[0] * residuals[0] + residuals[1] * residuals[1]) * 0.5);
            if (std::isfinite(line_rms) && line_rms <= threshold) {
                continue;
            }
            const double pixel_scale = cameras[camera_index].pixel_scale;
            stream << "Camera " << camera_index
                   << " / Line " << line_ids[line_index]
                   << ": RMS=" << line_rms
                   << " normalized px (" << line_rms / pixel_scale
                   << " source-image px), allowed=" << threshold
                   << " normalized px (" << threshold / pixel_scale
                   << " source-image px), confidence="
                   << cameras[camera_index].confidence * observation.confidence
                   << ".\n";
        }
    }
    return has_failure ? stream.str() : std::string{};
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
    for (Line3D &line : candidate->lines) {
        line.point *= *applied_scale;
        line.sample_extent *= *applied_scale;
    }
    for (std::size_t camera_index = 1; camera_index < candidate->poses.size(); ++camera_index) {
        const Mat3 rotation = candidate->poses[camera_index].R();
        const Vec3 center = candidate->poses[camera_index].center() * *applied_scale;
        candidate->poses[camera_index].t = -rotation * center;
    }
}

void FillOutputs(
    const Candidate &candidate,
    const CameraList &cameras,
    const PointObservationList &observations,
    const VisibilityList &point_visibility,
    const PointConfidenceList &point_confidences,
    const LineObservationList &line_observations,
    const VisibilityList &line_visibility,
    const std::int32_t *point_ids,
    const std::int32_t *line_ids,
    RT_CameraOutput *camera_outputs,
    RT_PointOutput *point_outputs,
    RT_LineOutput *line_outputs) {
    const Mat3 coordinate_flip = (Mat3() << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0).finished();

    for (std::size_t camera_index = 0; camera_index < candidate.poses.size(); ++camera_index) {
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
        double weight_sum = 0.0;
        for (std::size_t point_index = 0; point_index < candidate.points.size(); ++point_index) {
            if (!point_visibility[camera_index][point_index]) {
                continue;
            }
            const double confidence =
                cameras[camera_index].confidence *
                point_confidences[camera_index][point_index];
            squared_sum += confidence *
                           ReprojectionSquared(
                               candidate.points[point_index],
                               observations[camera_index][point_index],
                               cameras[camera_index],
                               candidate.focals[camera_index],
                               candidate.poses[camera_index]);
            weight_sum += confidence;
        }
        camera_outputs[camera_index].reprojection_rms_pixels =
            std::sqrt(squared_sum / std::max(1e-8, weight_sum * 2.0)) /
            cameras[camera_index].pixel_scale;
    }

    for (std::size_t point_index = 0; point_index < candidate.points.size(); ++point_index) {
        point_outputs[point_index].id = point_ids[point_index];
        const Vec3 point_unity = coordinate_flip * candidate.points[point_index];
        std::copy(point_unity.data(), point_unity.data() + 3, point_outputs[point_index].position);

        double squared_sum_pixels = 0.0;
        double weight_sum = 0.0;
        for (std::size_t camera_index = 0; camera_index < candidate.poses.size(); ++camera_index) {
            if (cameras[camera_index].pose_only ||
                !point_visibility[camera_index][point_index]) {
                continue;
            }
            const double squared_normalized = ReprojectionSquared(
                candidate.points[point_index],
                observations[camera_index][point_index],
                cameras[camera_index],
                candidate.focals[camera_index],
                candidate.poses[camera_index]);
            const double confidence =
                cameras[camera_index].confidence *
                point_confidences[camera_index][point_index];
            squared_sum_pixels += confidence * squared_normalized /
                                  (cameras[camera_index].pixel_scale * cameras[camera_index].pixel_scale);
            weight_sum += confidence;
        }
        point_outputs[point_index].reprojection_rms_pixels =
            std::sqrt(squared_sum_pixels / std::max(1e-8, weight_sum * 2.0));
    }

    for (std::size_t line_index = 0; line_index < candidate.lines.size(); ++line_index) {
        line_outputs[line_index].id = line_ids[line_index];
        const Vec3 point_unity = coordinate_flip * candidate.lines[line_index].point;
        const Vec3 direction_unity =
            (coordinate_flip * candidate.lines[line_index].direction).normalized();
        std::copy(point_unity.data(), point_unity.data() + 3, line_outputs[line_index].point);
        std::copy(
            direction_unity.data(),
            direction_unity.data() + 3,
            line_outputs[line_index].direction);

        double squared_sum_pixels = 0.0;
        double weight_sum = 0.0;
        for (std::size_t camera_index = 0; camera_index < candidate.poses.size(); ++camera_index) {
            if (cameras[camera_index].pose_only ||
                !line_visibility[camera_index][line_index]) {
                continue;
            }
            const NormalizedLineObservation &observation =
                line_observations[camera_index][line_index];
            const double confidence =
                cameras[camera_index].confidence * observation.confidence;
            const auto residuals = LineResiduals(
                candidate.lines[line_index],
                observation,
                cameras[camera_index],
                candidate.focals[camera_index],
                candidate.poses[camera_index]);
            const double inverse_pixel_scale = 1.0 / cameras[camera_index].pixel_scale;
            squared_sum_pixels +=
                confidence *
                (residuals[0] * residuals[0] + residuals[1] * residuals[1]) *
                inverse_pixel_scale * inverse_pixel_scale;
            weight_sum += confidence;
        }
        line_outputs[line_index].reprojection_rms_pixels =
            std::sqrt(squared_sum_pixels / std::max(1e-8, weight_sum * 2.0));
    }
}

} // namespace

const char *RT_CALL RT_GetVersion() {
    return "ReconstructionNative/1.8.0 (PoseLib 2.0.5, Ceres 2.2.0)";
}

std::int32_t RT_CALL RT_SolveMultiView(
    const RT_CameraInput *cameras,
    const std::int32_t camera_count,
    const std::int32_t *point_ids,
    const RT_Observation *observations,
    const std::uint8_t *observation_visibility,
    const double *observation_confidences,
    const std::int32_t point_count,
    const std::int32_t base_point_count,
    const std::int32_t *line_ids,
    const RT_LineObservation *line_observations,
    const std::uint8_t *line_observation_visibility,
    const std::int32_t line_count,
    const RT_SolveOptions *options,
    RT_CameraOutput *camera_outputs,
    RT_PointOutput *point_outputs,
    RT_LineOutput *line_outputs,
    RT_SolveReport *report,
    char *error_buffer,
    const std::int32_t error_buffer_capacity) {
    if (report != nullptr) {
        *report = {};
        report->status = RT_INTERNAL_ERROR;
    }
    WriteError(error_buffer, error_buffer_capacity, "");

    try {
        if (cameras == nullptr || point_ids == nullptr || observations == nullptr ||
            observation_visibility == nullptr || observation_confidences == nullptr ||
            options == nullptr ||
            camera_outputs == nullptr || point_outputs == nullptr || report == nullptr) {
            throw std::invalid_argument("One or more required pointers are null.");
        }
        if (camera_count < kBaseCameraCount || camera_count > kMaximumCameraCount) {
            throw std::invalid_argument("Camera count must be between 3 and 64.");
        }
        if (line_count < 0 ||
            (line_count > 0 &&
             (line_ids == nullptr || line_observations == nullptr ||
              line_observation_visibility == nullptr || line_outputs == nullptr))) {
            throw std::invalid_argument("Line count or line buffers are invalid.");
        }
        if (base_point_count < 8 || point_count < base_point_count) {
            throw std::invalid_argument(
                "At least 8 base reference points are required, and point count must include them.");
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
        unique_ids.clear();
        for (int line_index = 0; line_index < line_count; ++line_index) {
            if (!unique_ids.insert(line_ids[line_index]).second) {
                throw std::invalid_argument("Line IDs must be unique.");
            }
        }

        CameraList normalized_cameras(camera_count);
        PointObservationList normalized_observations(
            camera_count,
            std::vector<Vec2>(point_count, Vec2::Zero()));
        VisibilityList normalized_visibility(
            camera_count,
            std::vector<char>(point_count, 0));
        PointConfidenceList normalized_point_confidences(
            camera_count,
            std::vector<double>(point_count, 0.0));
        LineObservationList normalized_line_observations(
            camera_count,
            std::vector<NormalizedLineObservation>(line_count));
        VisibilityList normalized_line_visibility(
            camera_count,
            std::vector<char>(line_count, 0));
        for (int camera_index = 0; camera_index < camera_count; ++camera_index) {
            const auto &input = cameras[camera_index];
            if (input.pose_only != 0 && input.pose_only != 1) {
                throw std::invalid_argument("Camera pose-only mode must be 0 or 1.");
            }
            if (camera_index < kBaseCameraCount && input.pose_only != 0) {
                throw std::invalid_argument(
                    "Cameras 0, 1, and 2 cannot use pose-only mode.");
            }
            if (input.width <= 0 || input.height <= 0 ||
                input.min_vertical_fov_degrees <= 1.0 ||
                input.max_vertical_fov_degrees >= 179.0 ||
                input.min_vertical_fov_degrees >= input.max_vertical_fov_degrees ||
                !std::isfinite(input.confidence) ||
                input.confidence <= 0.0 ||
                input.confidence > 1.0) {
                throw std::invalid_argument("Camera dimensions, FOV bounds, or confidence are invalid.");
            }

            auto &camera = normalized_cameras[camera_index];
            camera.pixel_scale = kNormalizedLongSide / std::max(input.width, input.height);
            camera.width = input.width * camera.pixel_scale;
            camera.height = input.height * camera.pixel_scale;
            camera.principal = Vec2(camera.width * 0.5, camera.height * 0.5);
            camera.min_fov = input.min_vertical_fov_degrees;
            camera.max_fov = input.max_vertical_fov_degrees;
            camera.confidence = input.confidence;
            camera.pose_only = input.pose_only != 0;
            camera.min_focal = FocalFromVerticalFov(camera.height, camera.max_fov);
            camera.max_focal = FocalFromVerticalFov(camera.height, camera.min_fov);

            int visible_base_point_count = 0;
            for (int point_index = 0; point_index < point_count; ++point_index) {
                const int observation_index = camera_index * point_count + point_index;
                if (observation_visibility[observation_index] == 0) {
                    if (camera_index < kBaseCameraCount &&
                        point_index < base_point_count) {
                        throw std::invalid_argument(
                            "Cameras 0, 1, and 2 must observe every base reference point.");
                    }
                    continue;
                }
                if (camera_index < kBaseCameraCount &&
                    point_index >= base_point_count) {
                    throw std::invalid_argument(
                        "Additional reference points must be absent from Cameras 0, 1, and 2.");
                }
                const auto &point = observations[camera_index * point_count + point_index];
                const double point_confidence =
                    observation_confidences[observation_index];
                if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                    point.x < 0.0 || point.x > input.width ||
                    point.y < 0.0 || point.y > input.height ||
                    !std::isfinite(point_confidence) ||
                    point_confidence < 0.1 || point_confidence > 1.0) {
                    throw std::invalid_argument(
                        "A point observation, its bounds, or its confidence is invalid.");
                }
                normalized_observations[camera_index][point_index] =
                    Vec2(point.x * camera.pixel_scale, point.y * camera.pixel_scale);
                normalized_visibility[camera_index][point_index] = 1;
                normalized_point_confidences[camera_index][point_index] =
                    point_confidence;
                visible_base_point_count += point_index < base_point_count ? 1 : 0;
            }
            if (camera_index >= kBaseCameraCount && visible_base_point_count < 4) {
                throw std::invalid_argument(
                    "Each additional camera must observe at least 4 base reference points.");
            }

            for (int line_index = 0; line_index < line_count; ++line_index) {
                const int observation_index = camera_index * line_count + line_index;
                if (line_observation_visibility[observation_index] == 0) {
                    if (camera_index < kBaseCameraCount) {
                        throw std::invalid_argument(
                            "Cameras 0, 1, and 2 must observe every base reference line.");
                    }
                    continue;
                }
                const auto &line =
                    line_observations[observation_index];
                if (!std::isfinite(line.start_x) || !std::isfinite(line.start_y) ||
                    !std::isfinite(line.end_x) || !std::isfinite(line.end_y) ||
                    !std::isfinite(line.confidence) ||
                    line.confidence <= 0.0 || line.confidence > 1.0) {
                    throw std::invalid_argument(
                        "A line observation or its confidence is invalid.");
                }

                const double start_x = line.start_x * camera.pixel_scale;
                const double start_y = line.start_y * camera.pixel_scale;
                const double end_x = line.end_x * camera.pixel_scale;
                const double end_y = line.end_y * camera.pixel_scale;
                Vec3 equation(
                    start_y - end_y,
                    end_x - start_x,
                    start_x * end_y - end_x * start_y);
                const double normal_length = equation.head<2>().norm();
                if (!std::isfinite(normal_length) || normal_length < 20.0) {
                    throw std::invalid_argument(
                        "A line handle is too short; make its RectTransform wider.");
                }
                equation /= normal_length;
                normalized_line_observations[camera_index][line_index] = {
                    equation,
                    line.confidence};
                normalized_line_visibility[camera_index][line_index] = 1;
            }
        }

        for (int point_index = base_point_count; point_index < point_count; ++point_index) {
            int visible_additional_cameras = 0;
            for (int camera_index = kBaseCameraCount;
                 camera_index < camera_count;
                 ++camera_index) {
                visible_additional_cameras +=
                    !normalized_cameras[camera_index].pose_only &&
                    normalized_visibility[camera_index][point_index]
                        ? 1
                        : 0;
            }
            if (visible_additional_cameras < 2) {
                throw std::invalid_argument(
                    "Each additional reference point must be observed by at least 2 non-pose-only additional cameras.");
            }
        }

        PointObservationList base_observations(kBaseCameraCount);
        PointConfidenceList base_point_confidences(kBaseCameraCount);
        VisibilityList base_visibility(
            kBaseCameraCount,
            std::vector<char>(base_point_count, 1));
        for (int camera_index = 0; camera_index < kBaseCameraCount; ++camera_index) {
            base_observations[camera_index].assign(
                normalized_observations[camera_index].begin(),
                normalized_observations[camera_index].begin() + base_point_count);
            base_point_confidences[camera_index].assign(
                normalized_point_confidences[camera_index].begin(),
                normalized_point_confidences[camera_index].begin() + base_point_count);
        }

        bool planar_degenerate = false;
        const auto pairs = EstimatePairGeometry(
            base_observations, options->random_seed, &planar_degenerate);
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
            pairs,
            normalized_cameras,
            base_observations,
            base_visibility,
            base_point_confidences,
            options->random_seed);
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
        bool initialized_all_cameras = camera_count == kBaseCameraCount;
        int failed_additional_camera = -1;
        bool initialized_all_points = point_count == base_point_count;
        int failed_additional_point = -1;
        bool initialized_any_lines = line_count == 0;
        bool localized_all_pose_only_cameras = true;
        int failed_pose_only_camera = -1;
        for (Candidate &candidate : candidates) {
            // 先用基础三视图生成三维线，供附加相机初始化时参考。
            if (!InitializeCandidateLines(
                    &candidate,
                    normalized_cameras,
                    normalized_line_observations,
                    normalized_line_visibility)) {
                continue;
            }
            initialized_any_lines = true;

            int failed_camera_index = -1;
            if (!InitializeAdditionalCameras(
                    &candidate,
                    normalized_cameras,
                    normalized_observations,
                    normalized_visibility,
                    normalized_point_confidences,
                    normalized_line_observations,
                    normalized_line_visibility,
                    options->random_seed,
                    &failed_camera_index)) {
                failed_additional_camera = failed_camera_index;
                continue;
            }
            initialized_all_cameras = true;
            int failed_point_index = -1;
            if (!InitializeAdditionalPoints(
                    &candidate,
                    normalized_cameras,
                    normalized_observations,
                    normalized_visibility,
                    base_point_count,
                    &failed_point_index)) {
                failed_additional_point = failed_point_index;
                continue;
            }
            initialized_all_points = true;
            if (!InitializeCandidateLines(
                    &candidate,
                    normalized_cameras,
                    normalized_line_observations,
                    normalized_line_visibility)) {
                continue;
            }
            initialized_any_lines = true;
            if (OptimizeCandidate(
                    &candidate,
                    normalized_cameras,
                    normalized_observations,
                    normalized_visibility,
                    normalized_point_confidences,
                    normalized_line_observations,
                    normalized_line_visibility)) {
                int failed_pose_camera_index = -1;
                if (!OptimizePoseOnlyCameras(
                        &candidate,
                        normalized_cameras,
                        normalized_observations,
                        normalized_visibility,
                        normalized_point_confidences,
                        normalized_line_observations,
                        normalized_line_visibility,
                        &failed_pose_camera_index)) {
                    localized_all_pose_only_cameras = false;
                    failed_pose_only_camera = failed_pose_camera_index;
                    continue;
                }
                optimized.push_back(std::move(candidate));
            }
        }
        if (optimized.empty()) {
            report->status = RT_OPTIMIZATION_FAILED;
            std::string optimization_error;
            if (!initialized_all_cameras && failed_additional_camera >= 0) {
                int visible_base_points = 0;
                for (int point_index = 0;
                     point_index < base_point_count;
                     ++point_index) {
                    visible_base_points +=
                        normalized_visibility[failed_additional_camera][point_index]
                            ? 1
                            : 0;
                }
                int visible_lines = 0;
                for (int line_index = 0;
                     line_index < line_count;
                     ++line_index) {
                    visible_lines +=
                        normalized_line_visibility[failed_additional_camera][line_index]
                            ? 1
                            : 0;
                }
                optimization_error =
                    "Additional Camera " + std::to_string(failed_additional_camera) +
                    " could not be initialized or refined from " +
                    std::to_string(visible_base_points) +
                    " visible base points and " +
                    std::to_string(visible_lines) +
                    " reference lines. "
                    "Lines refine the point-based P4Pf initialization but cannot replace its "
                    "minimum 4 base points. Check point and line IDs, image coordinates, "
                    "point spread, and FOV bounds.";
            } else if (!initialized_all_points) {
                optimization_error =
                    "Additional point ID " +
                    std::to_string(point_ids[failed_additional_point]) +
                    " could not be triangulated. "
                    "Mark it in at least 2 non-pose-only additional cameras with enough parallax.";
            } else if (!localized_all_pose_only_cameras) {
                optimization_error =
                    "Pose-only Camera " +
                    std::to_string(failed_pose_only_camera) +
                    " could not be localized against the fixed 3D structure. "
                    "Check its point and line IDs, FOV bounds, and point spread.";
            } else if (initialized_any_lines) {
                optimization_error = "Bundle adjustment did not produce a usable solution.";
            } else {
                optimization_error =
                    "The reference lines cannot form stable 3D lines. "
                    "Check their IDs and viewing angles.";
            }
            WriteError(
                error_buffer,
                error_buffer_capacity,
                optimization_error);
            return report->status;
        }

        std::sort(optimized.begin(), optimized.end(), [](const Candidate &left, const Candidate &right) {
            return left.score < right.score;
        });
        Candidate best = std::move(optimized.front());

        // Preserve the best candidate so the editor can diagnose failed solves per image.
        FillOutputs(
            best,
            normalized_cameras,
            normalized_observations,
            normalized_visibility,
            normalized_point_confidences,
            normalized_line_observations,
            normalized_line_visibility,
            point_ids,
            line_ids,
            camera_outputs,
            point_outputs,
            line_outputs);

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
            normalized_visibility,
            normalized_point_confidences,
            point_ids,
            allowed_error);
        const std::string line_error = HighErrorLineMessage(
            best,
            normalized_cameras,
            normalized_line_observations,
            normalized_line_visibility,
            line_ids,
            allowed_error);
        const std::string pose_only_error = HighErrorPoseOnlyCameraMessage(
            best,
            normalized_cameras,
            normalized_observations,
            normalized_visibility,
            normalized_point_confidences,
            normalized_line_observations,
            normalized_line_visibility,
            point_ids,
            line_ids,
            allowed_error);
        if (!point_error.empty() || !line_error.empty() ||
            !pose_only_error.empty() ||
            best.rms > allowed_error || best.line_rms > allowed_error) {
            report->status = RT_HIGH_REPROJECTION_ERROR;
            std::string error = point_error;
            if (!line_error.empty()) {
                if (!error.empty()) {
                    error += "\n";
                }
                error += line_error;
            }
            if (!pose_only_error.empty()) {
                if (!error.empty()) {
                    error += "\n";
                }
                error += pose_only_error;
            }
            WriteError(
                error_buffer,
                error_buffer_capacity,
                error.empty() ? "The final reprojection error is too high." : error);
            return report->status;
        }

        for (int camera_index = 0; camera_index < camera_count; ++camera_index) {
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
            if (second.score <= best.score * 1.01 + 1e-6) {
                double maximum_fov_difference = 0.0;
                for (int camera_index = 0; camera_index < camera_count; ++camera_index) {
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
            normalized_visibility,
            normalized_point_confidences,
            normalized_line_observations,
            normalized_line_visibility,
            point_ids,
            line_ids,
            camera_outputs,
            point_outputs,
            line_outputs);

        report->status = RT_SUCCESS;
        report->point_count = point_count;
        report->inlier_count = point_count;
        report->line_count = line_count;
        report->normalized_reprojection_rms = best.rms;
        report->normalized_line_rms = best.line_rms;
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
