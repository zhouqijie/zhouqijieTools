using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

namespace ReconstructionTool.Editor
{
    internal static class ReconstructionCameraDiagnostics
    {
        /// <summary> 根据最佳候选的逐图片残差生成广角和畸变嫌疑报告。 </summary>
        internal static string Build(
            ReconstructionInput input,
            ReconstructionNativeApi.CameraOutput[] cameras,
            ReconstructionNativeApi.PointOutput[] points,
            out bool hasWarning)
        {
            hasWarning = false;
            if (input == null || cameras == null || points == null ||
                cameras.Length != input.Cameras.Length ||
                points.Length != input.PointIds.Length)
            {
                return string.Empty;
            }

            var details = new StringBuilder();
            var normalizedRmsByCamera = new double[cameras.Length];
            var sourceRmsByCamera = new double[cameras.Length];
            var validCamera = new bool[cameras.Length];
            var highResidualByCamera = new bool[cameras.Length];
            var distortionByCamera = new bool[cameras.Length];
            bool hasCandidate = false;
            for (int cameraIndex = 0; cameraIndex < cameras.Length; cameraIndex++)
            {
                ReconstructionNativeApi.CameraOutput camera = cameras[cameraIndex];
                ReconstructionNativeApi.CameraInput cameraInput = input.Cameras[cameraIndex];
                if (!(camera.FocalLengthPixels > 0.0) ||
                    double.IsNaN(camera.FocalLengthPixels) ||
                    double.IsInfinity(camera.FocalLengthPixels))
                {
                    continue;
                }

                Quaternion rotation = new(
                    (float)camera.RotationX,
                    (float)camera.RotationY,
                    (float)camera.RotationZ,
                    (float)camera.RotationW);
                float rotationSquaredMagnitude =
                    rotation.x * rotation.x +
                    rotation.y * rotation.y +
                    rotation.z * rotation.z +
                    rotation.w * rotation.w;
                if (rotationSquaredMagnitude < 0.5f)
                {
                    continue;
                }

                hasCandidate = true;
                rotation = Quaternion.Normalize(rotation);
                Quaternion inverseRotation = Quaternion.Inverse(rotation);
                Vector3 cameraPosition = new(
                    (float)camera.PositionX,
                    (float)camera.PositionY,
                    (float)camera.PositionZ);
                double pixelScale = 1000.0 / Math.Max(cameraInput.Width, cameraInput.Height);
                double halfDiagonal = 0.5 * Math.Sqrt(
                    cameraInput.Width * (double)cameraInput.Width +
                    cameraInput.Height * (double)cameraInput.Height);

                // 计算原始像素残差及其径向、切向分量。
                var radiusCubed = new List<double>(points.Length);
                var radialResiduals = new List<double>(points.Length);
                var pointErrors = new List<(int Id, double Error)>(points.Length);
                double radialSquaredSum = 0.0;
                double tangentialSquaredSum = 0.0;
                double residualSquaredSum = 0.0;
                double residualXSum = 0.0;
                double residualYSum = 0.0;
                double centerSquaredSum = 0.0;
                double edgeSquaredSum = 0.0;
                double minimumRadius = double.MaxValue;
                double maximumRadius = double.MinValue;
                int validCount = 0;
                int centerCount = 0;
                int edgeCount = 0;

                for (int pointIndex = 0; pointIndex < points.Length; pointIndex++)
                {
                    int observationIndex = cameraIndex * points.Length + pointIndex;
                    if (input.ObservationVisibility[observationIndex] == 0)
                    {
                        continue;
                    }

                    ReconstructionNativeApi.PointOutput point = points[pointIndex];
                    Vector3 pointPosition = new(
                        (float)point.PositionX,
                        (float)point.PositionY,
                        (float)point.PositionZ);
                    Vector3 cameraPoint = inverseRotation * (pointPosition - cameraPosition);
                    if (cameraPoint.z <= 0.000001f)
                    {
                        continue;
                    }

                    double projectedX =
                        camera.FocalLengthPixels * cameraPoint.x / cameraPoint.z +
                        cameraInput.Width * 0.5;
                    double projectedY =
                        cameraInput.Height * 0.5 -
                        camera.FocalLengthPixels * cameraPoint.y / cameraPoint.z;
                    ReconstructionNativeApi.Observation observation =
                        input.Observations[observationIndex];
                    double residualX = (observation.X - projectedX) * pixelScale;
                    double residualY = (observation.Y - projectedY) * pixelScale;
                    double squaredResidual = residualX * residualX + residualY * residualY;
                    residualSquaredSum += squaredResidual;
                    residualXSum += residualX;
                    residualYSum += residualY;
                    pointErrors.Add((
                        input.PointIds[pointIndex],
                        Math.Sqrt(squaredResidual) / pixelScale));
                    validCount++;

                    double offsetX = observation.X - cameraInput.Width * 0.5;
                    double offsetY = observation.Y - cameraInput.Height * 0.5;
                    double radiusPixels = Math.Sqrt(offsetX * offsetX + offsetY * offsetY);
                    if (radiusPixels <= 0.000001)
                    {
                        continue;
                    }

                    double radius = radiusPixels / halfDiagonal;
                    double directionX = offsetX / radiusPixels;
                    double directionY = offsetY / radiusPixels;
                    double radial = residualX * directionX + residualY * directionY;
                    double tangential = -residualX * directionY + residualY * directionX;
                    radiusCubed.Add(radius * radius * radius);
                    radialResiduals.Add(radial);
                    radialSquaredSum += radial * radial;
                    tangentialSquaredSum += tangential * tangential;
                    minimumRadius = Math.Min(minimumRadius, radius);
                    maximumRadius = Math.Max(maximumRadius, radius);

                    if (radius <= 0.35)
                    {
                        centerSquaredSum += squaredResidual;
                        centerCount++;
                    }
                    else if (radius >= 0.65)
                    {
                        edgeSquaredSum += squaredResidual;
                        edgeCount++;
                    }
                }

                if (validCount == 0)
                {
                    continue;
                }

                // 依据 FOV 与残差随画面半径的变化评估风险。
                double normalizedRms = Math.Sqrt(residualSquaredSum / (validCount * 2.0));
                double sourcePixelRms = normalizedRms / pixelScale;
                double diagonalFov = 2.0 * Math.Atan(
                    Math.Sqrt(
                        cameraInput.Width * (double)cameraInput.Width +
                        cameraInput.Height * (double)cameraInput.Height) /
                    (2.0 * camera.FocalLengthPixels)) * Mathf.Rad2Deg;
                bool ultraWide =
                    diagonalFov >= 110.0 || camera.HorizontalFov >= 100.0 || camera.VerticalFov >= 85.0;
                bool wide =
                    ultraWide || diagonalFov >= 95.0 ||
                    camera.HorizontalFov >= 85.0 || camera.VerticalFov >= 70.0;
                bool enoughRadialCoverage =
                    radiusCubed.Count >= 8 && maximumRadius - minimumRadius >= 0.35;
                double radialCorrelation = enoughRadialCoverage
                    ? Correlation(radiusCubed, radialResiduals)
                    : 0.0;
                double radialRms = radiusCubed.Count > 0
                    ? Math.Sqrt(radialSquaredSum / radiusCubed.Count)
                    : 0.0;
                double tangentialRms = radiusCubed.Count > 0
                    ? Math.Sqrt(tangentialSquaredSum / radiusCubed.Count)
                    : 0.0;
                bool hasEdgeRatio = centerCount >= 3 && edgeCount >= 3;
                double centerRms = hasEdgeRatio
                    ? Math.Sqrt(centerSquaredSum / (centerCount * 2.0))
                    : 0.0;
                double edgeRms = hasEdgeRatio
                    ? Math.Sqrt(edgeSquaredSum / (edgeCount * 2.0))
                    : 0.0;
                double edgeRatio = hasEdgeRatio
                    ? edgeRms / Math.Max(0.05, centerRms)
                    : 0.0;
                double radialSlopeNumerator = 0.0;
                double radialSlopeDenominator = 0.0;
                for (int index = 0; index < radiusCubed.Count; index++)
                {
                    radialSlopeNumerator += radiusCubed[index] * radialResiduals[index];
                    radialSlopeDenominator += radiusCubed[index] * radiusCubed[index];
                }
                double radialSlope = radialSlopeNumerator /
                                     Math.Max(0.00000001, radialSlopeDenominator);
                bool canEstimateDistortion =
                    radiusCubed.Count >= 5 && maximumRadius - minimumRadius >= 0.25;
                double estimatedOuterRadialNormalized = canEstimateDistortion
                    ? radialSlope * maximumRadius * maximumRadius * maximumRadius
                    : 0.0;
                double estimatedOuterRadialSource =
                    estimatedOuterRadialNormalized / pixelScale;
                double estimatedOuterLongSidePercent =
                    Math.Abs(estimatedOuterRadialNormalized) * 0.1;
                double radialDominance =
                    radialRms / Math.Max(0.05, tangentialRms);
                double tolerance = input.Options.MaximumNormalizedReprojectionError;
                bool suspectedDistortion =
                    enoughRadialCoverage &&
                    Math.Abs(radialCorrelation) >= 0.65 &&
                    radialRms >= Math.Max(0.5, tangentialRms * 1.3) &&
                    normalizedRms >= Math.Max(0.75, tolerance * 0.45) &&
                    (!hasEdgeRatio || edgeRatio >= 1.4);
                bool moderateDistortionEvidence =
                    enoughRadialCoverage &&
                    Math.Abs(radialCorrelation) >= 0.5 &&
                    radialDominance >= 1.1 &&
                    Math.Abs(estimatedOuterRadialNormalized) >= 0.5 &&
                    (!hasEdgeRatio || edgeRatio >= 1.15);
                bool highResidual = normalizedRms > tolerance || normalizedRms > 3.0;

                // 判断误差是集中在少数点，还是整张图片存在系统性偏移。
                pointErrors.Sort((left, right) => right.Error.CompareTo(left.Error));
                double totalSourceSquared = residualSquaredSum / (pixelScale * pixelScale);
                double topSourceSquared = 0.0;
                for (int index = 0; index < Math.Min(3, pointErrors.Count); index++)
                {
                    topSourceSquared += pointErrors[index].Error * pointErrors[index].Error;
                }
                bool localizedPointErrors =
                    topSourceSquared / Math.Max(0.000001, totalSourceSquared) >= 0.55;
                double meanResidualMagnitude = Math.Sqrt(
                    residualXSum * residualXSum + residualYSum * residualYSum) / validCount;
                double vectorRms = Math.Sqrt(residualSquaredSum / validCount);
                bool systematicOffset =
                    meanResidualMagnitude >= Math.Max(0.75, vectorRms * 0.55);
                bool atMinimumFov =
                    Math.Abs(camera.VerticalFov - cameraInput.MinimumVerticalFov) <= 0.25;
                bool atMaximumFov =
                    Math.Abs(camera.VerticalFov - cameraInput.MaximumVerticalFov) <= 0.25;

                validCamera[cameraIndex] = true;
                normalizedRmsByCamera[cameraIndex] = normalizedRms;
                sourceRmsByCamera[cameraIndex] = sourcePixelRms;
                highResidualByCamera[cameraIndex] = highResidual;
                distortionByCamera[cameraIndex] = suspectedDistortion;
                hasWarning |= ultraWide || highResidual || suspectedDistortion ||
                              atMinimumFov || atMaximumFov;

                // 先输出人能直接采取行动的结论。
                string pictureName = input.Pictures[cameraIndex] != null
                    ? input.Pictures[cameraIndex].name
                    : $"Camera {cameraIndex}";
                details.AppendLine();
                details.AppendLine($"Camera {cameraIndex}「{pictureName}」");
                if (highResidual)
                {
                    double sourceTolerance = tolerance / pixelScale;
                    details.AppendLine(
                        $"  结论：有问题。综合重投影误差约 {sourcePixelRms:F1}px，" +
                        $"当前允许值约 {sourceTolerance:F1}px。");
                    if (suspectedDistortion)
                    {
                        details.AppendLine(
                            "  更像什么：误差随画面半径明显增长，疑似未校正的镜头或全景畸变。");
                    }
                    else if (localizedPointErrors)
                    {
                        details.AppendLine(
                            "  更像什么：主要由少数几个点拖累，优先怀疑点标偏、ID 对错物体或物体发生变化；" +
                            "不像典型的镜头径向畸变。");
                    }
                    else if (systematicOffset)
                    {
                        details.AppendLine(
                            "  更像什么：多数点整体向同一方向偏移，优先检查图片是否被裁剪、留黑边，" +
                            "或 RectTransform 尺寸与原图像素不一致；不像典型的镜头径向畸变。");
                    }
                    else
                    {
                        details.AppendLine(
                            "  更像什么：多处点都无法与另外两张图同时对齐，优先检查同 ID 是否真是同一物理点、" +
                            "图片是否被裁剪或非等比拉伸；目前不像典型的镜头径向畸变。");
                    }

                    details.Append("  偏差最大的点（优先检查）：");
                    for (int index = 0; index < Math.Min(5, pointErrors.Count); index++)
                    {
                        details.Append(index == 0 ? " " : "、");
                        details.Append($"ID {pointErrors[index].Id}≈{pointErrors[index].Error:F1}px");
                    }
                    details.AppendLine("。");

                    double suggestedConfidence = suspectedDistortion
                        ? ultraWide ? 0.25 : 0.4
                        : 0.5;
                    details.AppendLine(
                        "  建议：先修正上述点和图片尺寸，不要先靠增大容差强行通过；" +
                        $"确实无法换图时，再尝试置信度≈{suggestedConfidence:F2}。");
                }
                else
                {
                    details.AppendLine(
                        $"  结论：可以使用。综合重投影误差约 {sourcePixelRms:F1}px，未超过容差。");
                    details.AppendLine(wide
                        ? "  镜头判断：虽然属于广角，但目前没有明显的畸变残差模式，不需要仅因为广角而降权。"
                        : "  镜头判断：没有发现明显的广角或畸变问题。");
                }

                // 单独输出每张图片的残差型畸变估计。
                if (canEstimateDistortion)
                {
                    string distortionEvidence = suspectedDistortion
                        ? "高"
                        : moderateDistortionEvidence
                            ? "中"
                            : enoughRadialCoverage ? "低" : "不确定";
                    string coverageReliability =
                        radiusCubed.Count >= 15 &&
                        maximumRadius - minimumRadius >= 0.55 &&
                        hasEdgeRatio
                            ? "较高"
                            : enoughRadialCoverage ? "中" : "低";
                    string distortionDirection =
                        Math.Abs(radialCorrelation) < 0.4 ||
                        Math.Abs(estimatedOuterRadialSource) < 0.5
                            ? "没有稳定的径向方向"
                            : estimatedOuterRadialSource < 0.0
                                ? "偏向画面中心（桶形倾向）"
                                : "偏向画面外侧（枕形倾向）";
                    details.AppendLine(
                        "  畸变估计（非镜头标定值）：" +
                        $"畸变证据={distortionEvidence}，" +
                        $"最外圈参考点的系统性径向偏移约 " +
                        $"{Math.Abs(estimatedOuterRadialSource):F1}px" +
                        $"（图片长边 {estimatedOuterLongSidePercent:F2}%），" +
                        $"{distortionDirection}；点位覆盖可信度={coverageReliability}。");
                }
                else
                {
                    details.AppendLine(
                        "  畸变估计：参考点数量或中心到边缘的覆盖不足，无法可靠估计；" +
                        "建议至少使用 5 个分散点，8 个以上更可靠。");
                }

                if (atMinimumFov || atMaximumFov)
                {
                    string boundary = atMinimumFov ? "最小值" : "最大值";
                    details.AppendLine(
                        $"  FOV 提醒：解算结果卡在垂直 FOV {boundary} " +
                        $"{camera.VerticalFov:F1}°，建议适当放宽该边界后重算。");
                }

                // 技术数据保留在最后，便于进一步排查。
                string viewType = ultraWide ? "超广角" : wide ? "广角" : "常规";
                string solveMode = input.Pictures[cameraIndex] != null &&
                                   input.Pictures[cameraIndex].CameraPoseOnly
                    ? "仅求相机位姿"
                    : "参与公共重建";
                details.Append(
                    $"  技术数据：视角={viewType}，HFOV={camera.HorizontalFov:F1}°，" +
                    $"VFOV={camera.VerticalFov:F1}°，DFOV={diagonalFov:F1}°，" +
                    $"归一化 RMS={normalizedRms:F2}，模式={solveMode}，" +
                    $"配置置信度={cameraInput.Confidence:F2}，");
                details.Append(enoughRadialCoverage
                    ? $"径向相关={radialCorrelation:F2}" +
                      (hasEdgeRatio ? $"，边缘/中心残差={edgeRatio:F2}" : string.Empty)
                    : "径向检测点位覆盖不足");
                details.AppendLine("。");
            }

            if (!hasCandidate)
            {
                return string.Empty;
            }

            // 在最前面给出整组图片的直白结论。
            int worstCameraIndex = -1;
            for (int cameraIndex = 0; cameraIndex < cameras.Length; cameraIndex++)
            {
                if (validCamera[cameraIndex] &&
                    (worstCameraIndex < 0 ||
                     normalizedRmsByCamera[cameraIndex] > normalizedRmsByCamera[worstCameraIndex]))
                {
                    worstCameraIndex = cameraIndex;
                }
            }

            var report = new StringBuilder("[ReconstructionTool] 图片质量诊断");
            report.AppendLine();
            if (worstCameraIndex >= 0 && highResidualByCamera[worstCameraIndex])
            {
                double otherRmsSum = 0.0;
                int otherCameraCount = 0;
                for (int cameraIndex = 0; cameraIndex < cameras.Length; cameraIndex++)
                {
                    if (cameraIndex != worstCameraIndex && validCamera[cameraIndex])
                    {
                        otherRmsSum += normalizedRmsByCamera[cameraIndex];
                        otherCameraCount++;
                    }
                }

                string worstPictureName = input.Pictures[worstCameraIndex] != null
                    ? input.Pictures[worstCameraIndex].name
                    : $"Camera {worstCameraIndex}";
                report.Append(
                    $"总判断：主要问题是 Camera {worstCameraIndex}「{worstPictureName}」，" +
                    $"其综合偏差约 {sourceRmsByCamera[worstCameraIndex]:F1}px");
                if (otherCameraCount > 0)
                {
                    double relativeError = normalizedRmsByCamera[worstCameraIndex] /
                                           Math.Max(0.01, otherRmsSum / otherCameraCount);
                    report.Append($"，约为另外图片平均水平的 {relativeError:F1} 倍");
                }
                report.AppendLine("。");
                report.AppendLine(distortionByCamera[worstCameraIndex]
                    ? "它呈现出镜头/全景畸变模式，请查看下面的具体点位。"
                    : "它目前不像典型镜头畸变，更应先检查标点、ID 对应、裁剪和图片尺寸。");
            }
            else
            {
                report.AppendLine("总判断：没有图片超过当前重投影容差；若仍无法通过，请重点查看下面的 FOV 边界提醒。");
            }

            report.Append(details);
            report.AppendLine();
            report.Append(
                "说明：像素偏差越小越好；优先修正列出的点。降低置信度只能止损，" +
                "不能修复标错、裁剪、拉伸或镜头畸变。");
            return report.ToString();
        }

        /// <summary> 计算两组数值的皮尔逊相关系数。 </summary>
        private static double Correlation(IReadOnlyList<double> left, IReadOnlyList<double> right)
        {
            double leftMean = 0.0;
            double rightMean = 0.0;
            for (int index = 0; index < left.Count; index++)
            {
                leftMean += left[index];
                rightMean += right[index];
            }
            leftMean /= left.Count;
            rightMean /= right.Count;

            double covariance = 0.0;
            double leftVariance = 0.0;
            double rightVariance = 0.0;
            for (int index = 0; index < left.Count; index++)
            {
                double leftDelta = left[index] - leftMean;
                double rightDelta = right[index] - rightMean;
                covariance += leftDelta * rightDelta;
                leftVariance += leftDelta * leftDelta;
                rightVariance += rightDelta * rightDelta;
            }

            double denominator = Math.Sqrt(leftVariance * rightVariance);
            return denominator > 0.00000001 ? covariance / denominator : 0.0;
        }
    }
}
