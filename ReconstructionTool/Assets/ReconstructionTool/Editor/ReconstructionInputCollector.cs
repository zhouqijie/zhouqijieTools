using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace ReconstructionTool.Editor
{
    internal sealed class ReconstructionInput
    {
        internal RefPicture[] Pictures { get; }
        internal ReconstructionNativeApi.CameraInput[] Cameras { get; }
        internal int[] PointIds { get; }
        internal int BasePointCount { get; }
        internal ReconstructionNativeApi.Observation[] Observations { get; }
        internal byte[] ObservationVisibility { get; }
        internal double[] ObservationConfidences { get; }
        internal int[] LineIds { get; }
        internal ReconstructionNativeApi.LineObservation[] LineObservations { get; }
        internal byte[] LineObservationVisibility { get; }
        internal int? ResultXAxisLineId { get; }
        internal ReconstructionNativeApi.SolveOptions Options { get; }
        internal string Warning { get; }

        internal ReconstructionInput(
            RefPicture[] pictures,
            ReconstructionNativeApi.CameraInput[] cameras,
            int[] pointIds,
            int basePointCount,
            ReconstructionNativeApi.Observation[] observations,
            byte[] observationVisibility,
            double[] observationConfidences,
            int[] lineIds,
            ReconstructionNativeApi.LineObservation[] lineObservations,
            byte[] lineObservationVisibility,
            int? resultXAxisLineId,
            ReconstructionNativeApi.SolveOptions options,
            string warning)
        {
            Pictures = pictures;
            Cameras = cameras;
            PointIds = pointIds;
            BasePointCount = basePointCount;
            Observations = observations;
            ObservationVisibility = observationVisibility;
            ObservationConfidences = observationConfidences;
            LineIds = lineIds;
            LineObservations = lineObservations;
            LineObservationVisibility = lineObservationVisibility;
            ResultXAxisLineId = resultXAxisLineId;
            Options = options;
            Warning = warning;
        }
    }

    internal static class ReconstructionInputCollector
    {
        internal static bool TryCollect(out ReconstructionInput input, out string error)
        {
            input = null;
            error = string.Empty;

            RefPicture[] pictures = UnityEngine.Object
                .FindObjectsByType<RefPicture>(FindObjectsInactive.Include, FindObjectsSortMode.None)
                .Where(picture => picture.gameObject.scene.IsValid())
                .ToArray();
            if (pictures.Length < 3)
            {
                error = $"场景中至少需要 3 个 RefPicture，当前找到 {pictures.Length} 个。";
                return false;
            }
            if (pictures.Length > 64)
            {
                error = $"最多支持 64 个 RefPicture，当前找到 {pictures.Length} 个。";
                return false;
            }

            if (pictures.Select(picture => picture.CameraId).Distinct().Count() != pictures.Length ||
                pictures.Any(picture => picture.CameraId < 0))
            {
                error = "所有 RefPicture 的 Camera ID 必须是唯一的非负整数。";
                return false;
            }

            Array.Sort(pictures, (left, right) => left.CameraId.CompareTo(right.CameraId));
            for (int cameraIndex = 0; cameraIndex < pictures.Length; cameraIndex++)
            {
                if (pictures[cameraIndex].CameraId != cameraIndex)
                {
                    error = $"Camera ID 必须从 0 连续编号到 {pictures.Length - 1}，" +
                            $"当前缺少 ID {cameraIndex}。";
                    return false;
                }
            }
            if (pictures.Take(3).Any(picture => picture.CameraPoseOnly))
            {
                error = "Camera 0～2 是基础三视图，不能设为“仅求相机位姿”。";
                return false;
            }

            var pointMaps = new Dictionary<int, RefPoint2D>[pictures.Length];
            var lineMaps = new Dictionary<int, RefLine2D>[pictures.Length];
            for (int cameraIndex = 0; cameraIndex < pictures.Length; cameraIndex++)
            {
                RefPicture picture = pictures[cameraIndex];
                if (!TryGetPictureSize(picture, out int width, out int height, out error))
                {
                    return false;
                }

                pointMaps[cameraIndex] = new Dictionary<int, RefPoint2D>();
                foreach (RefPoint2D point in picture.GetComponentsInChildren<RefPoint2D>(true))
                {
                    if (!pointMaps[cameraIndex].TryAdd(point.Id, point))
                    {
                        error = $"Camera {cameraIndex} 中 RefPoint2D ID {point.Id} 重复。";
                        return false;
                    }

                    Vector2 pixel = WorldToTopLeftPixel(picture.RectTransform, point.transform.position);
                    const float epsilon = 0.001f;
                    if (pixel.x < -epsilon || pixel.y < -epsilon ||
                        pixel.x > width + epsilon || pixel.y > height + epsilon)
                    {
                        error = $"Camera {cameraIndex} 的点 ID {point.Id} 位于图片矩形外：" +
                                $"({pixel.x:F2}, {pixel.y:F2}) / {width}×{height}。";
                        return false;
                    }
                }

                lineMaps[cameraIndex] = new Dictionary<int, RefLine2D>();
                foreach (RefLine2D line in picture.GetComponentsInChildren<RefLine2D>(true))
                {
                    if (!lineMaps[cameraIndex].TryAdd(line.Id, line))
                    {
                        error = $"Camera {cameraIndex} 中 RefLine2D ID {line.Id} 重复。";
                        return false;
                    }

                    line.GetWorldEndpoints(out Vector3 worldStart, out Vector3 worldEnd);
                    Vector2 start = WorldToTopLeftPixel(picture.RectTransform, worldStart);
                    Vector2 end = WorldToTopLeftPixel(picture.RectTransform, worldEnd);
                    float normalizedLength =
                        Vector2.Distance(start, end) * 1000f / Mathf.Max(width, height);
                    if (normalizedLength < 20f)
                    {
                        error = $"Camera {cameraIndex} 的参考线 ID {line.Id} 操作段太短。" +
                                "请把 RefLine2D 的 RectTransform 宽度拉长，使方向更容易准确对齐。";
                        return false;
                    }
                    if (!LineIntersectsPicture(start, end, width, height))
                    {
                        error = $"Camera {cameraIndex} 的参考线 ID {line.Id} 没有穿过图片。" +
                                "请把粉色虚线移动到实际可见的直边上。";
                        return false;
                    }
                }
            }

            int[] basePointIds = pointMaps[0].Keys.OrderBy(id => id).ToArray();
            if (basePointIds.Length < 8)
            {
                error = $"至少需要 8 个共同参考点，当前只有 {basePointIds.Length} 个。";
                return false;
            }

            for (int cameraIndex = 1; cameraIndex < 3; cameraIndex++)
            {
                if (!basePointIds.SequenceEqual(pointMaps[cameraIndex].Keys.OrderBy(id => id)))
                {
                    error = $"Camera {cameraIndex} 的 RefPoint2D ID 集合与 Camera 0 不一致。";
                    return false;
                }
            }

            var basePointIdSet = new HashSet<int>(basePointIds);
            for (int cameraIndex = 3; cameraIndex < pictures.Length; cameraIndex++)
            {
                int visibleBasePointCount =
                    pointMaps[cameraIndex].Keys.Count(basePointIdSet.Contains);
                if (visibleBasePointCount < 4)
                {
                    error = $"附加 Camera {cameraIndex} 至少需要标出 4 个基础参考点，" +
                            $"当前只有 {visibleBasePointCount} 个；附加新点不能用于初始化相机。";
                    return false;
                }
            }

            int[] reconstructionAdditionalCameras = Enumerable
                .Range(3, pictures.Length - 3)
                .Where(cameraIndex => !pictures[cameraIndex].CameraPoseOnly)
                .ToArray();
            int[] additionalPointIds = reconstructionAdditionalCameras
                .SelectMany(cameraIndex => pointMaps[cameraIndex].Keys)
                .Where(id => !basePointIdSet.Contains(id))
                .Distinct()
                .OrderBy(id => id)
                .ToArray();
            foreach (int pointId in additionalPointIds)
            {
                int visibleAdditionalCameras = reconstructionAdditionalCameras
                    .Count(cameraIndex => pointMaps[cameraIndex].ContainsKey(pointId));
                if (visibleAdditionalCameras < 2)
                {
                    error = $"附加参考点 ID {pointId} 只在 {visibleAdditionalCameras} 个附加机位中可见。" +
                            "它至少需要被两个附加机位使用相同 ID 标记，" +
                            "且这两个机位都必须参与公共重建，才能恢复三维位置。";
                    return false;
                }
            }
            int[] pointIds = basePointIds.Concat(additionalPointIds).ToArray();
            var reconstructedPointIdSet = new HashSet<int>(pointIds);
            for (int cameraIndex = 3; cameraIndex < pictures.Length; cameraIndex++)
            {
                if (!pictures[cameraIndex].CameraPoseOnly)
                {
                    continue;
                }
                int unknownPointId = pointMaps[cameraIndex].Keys.FirstOrDefault(
                    id => !reconstructedPointIdSet.Contains(id));
                if (pointMaps[cameraIndex].Keys.Any(id => !reconstructedPointIdSet.Contains(id)))
                {
                    error = $"仅求相机位姿的 Camera {cameraIndex} 使用了点 ID {unknownPointId}，" +
                            "但该点尚未由参与公共重建的机位生成。";
                    return false;
                }
            }

            int[] lineIds = lineMaps[0].Keys.OrderBy(id => id).ToArray();
            for (int cameraIndex = 1; cameraIndex < 3; cameraIndex++)
            {
                if (!lineIds.SequenceEqual(lineMaps[cameraIndex].Keys.OrderBy(id => id)))
                {
                    error = $"Camera {cameraIndex} 的 RefLine2D ID 集合与 Camera 0 不一致。";
                    return false;
                }
            }

            for (int cameraIndex = 3; cameraIndex < pictures.Length; cameraIndex++)
            {
                int unknownLineId = lineMaps[cameraIndex].Keys.FirstOrDefault(
                    id => Array.BinarySearch(lineIds, id) < 0);
                if (lineMaps[cameraIndex].Keys.Any(id => Array.BinarySearch(lineIds, id) < 0))
                {
                    error = $"附加 Camera {cameraIndex} 的参考线 ID {unknownLineId} " +
                            "不在 Camera 0～2 的基础参考线集中。";
                    return false;
                }
            }

            int[] resultXAxisLineIds = lineMaps
                .SelectMany(map => map.Values)
                .Where(line => line.UseAsResultXAxis)
                .Select(line => line.Id)
                .Distinct()
                .ToArray();
            if (resultXAxisLineIds.Length > 1)
            {
                error = "只能指定一个参考线 Line ID 作为结果 X 轴。";
                return false;
            }
            int? resultXAxisLineId = resultXAxisLineIds.Length == 1
                ? resultXAxisLineIds[0]
                : null;

            RefPicture scalePicture = pictures[0];
            if (scalePicture.ScalePointIdA == scalePicture.ScalePointIdB)
            {
                error = "Camera 0 的两个尺度参考点 ID 必须不同。";
                return false;
            }
            if (!pointMaps[0].ContainsKey(scalePicture.ScalePointIdA) ||
                !pointMaps[0].ContainsKey(scalePicture.ScalePointIdB))
            {
                error = "Camera 0 的尺度参考点 ID 必须存在于三张图片的共同点集中。";
                return false;
            }
            if (!(scalePicture.ScaleReferenceDistance > 0f))
            {
                error = "尺度参考真实距离必须大于 0。";
                return false;
            }

            var cameras = new ReconstructionNativeApi.CameraInput[pictures.Length];
            var observations =
                new ReconstructionNativeApi.Observation[pictures.Length * pointIds.Length];
            var observationVisibility = new byte[observations.Length];
            var observationConfidences = new double[observations.Length];
            var lineObservations =
                new ReconstructionNativeApi.LineObservation[pictures.Length * lineIds.Length];
            var lineObservationVisibility = new byte[lineObservations.Length];
            for (int cameraIndex = 0; cameraIndex < pictures.Length; cameraIndex++)
            {
                RefPicture picture = pictures[cameraIndex];
                TryGetPictureSize(picture, out int width, out int height, out _);
                cameras[cameraIndex] = new ReconstructionNativeApi.CameraInput
                {
                    Width = width,
                    Height = height,
                    MinimumVerticalFov = picture.MinimumVerticalFov,
                    MaximumVerticalFov = picture.MaximumVerticalFov,
                    Confidence = picture.Confidence,
                    PoseOnly = picture.CameraPoseOnly ? 1 : 0
                };

                for (int pointIndex = 0; pointIndex < pointIds.Length; pointIndex++)
                {
                    if (!pointMaps[cameraIndex].TryGetValue(
                            pointIds[pointIndex],
                            out RefPoint2D point))
                    {
                        continue;
                    }
                    Vector2 pixel = WorldToTopLeftPixel(picture.RectTransform, point.transform.position);
                    int observationIndex = cameraIndex * pointIds.Length + pointIndex;
                    observations[observationIndex] =
                        new ReconstructionNativeApi.Observation { X = pixel.x, Y = pixel.y };
                    observationVisibility[observationIndex] = 1;
                    observationConfidences[observationIndex] = point.Confidence;
                }

                for (int lineIndex = 0; lineIndex < lineIds.Length; lineIndex++)
                {
                    if (!lineMaps[cameraIndex].TryGetValue(
                            lineIds[lineIndex],
                            out RefLine2D line))
                    {
                        continue;
                    }
                    line.GetWorldEndpoints(out Vector3 worldStart, out Vector3 worldEnd);
                    Vector2 start = WorldToTopLeftPixel(picture.RectTransform, worldStart);
                    Vector2 end = WorldToTopLeftPixel(picture.RectTransform, worldEnd);
                    int observationIndex = cameraIndex * lineIds.Length + lineIndex;
                    lineObservations[observationIndex] =
                        new ReconstructionNativeApi.LineObservation
                        {
                            StartX = start.x,
                            StartY = start.y,
                            EndX = end.x,
                            EndY = end.y,
                            Confidence = line.Confidence
                        };
                    lineObservationVisibility[observationIndex] = 1;
                }
            }

            var options = new ReconstructionNativeApi.SolveOptions
            {
                ScalePointIdA = scalePicture.ScalePointIdA,
                ScalePointIdB = scalePicture.ScalePointIdB,
                KnownScaleDistance = scalePicture.ScaleReferenceDistance,
                MaximumNormalizedReprojectionError =
                    scalePicture.MaximumNormalizedReprojectionError,
                RandomSeed = 1337,
                MaximumCandidates = 12
            };
            string warning = basePointIds.Length < 15
                ? $"只有 {basePointIds.Length} 个共同基础点，结果可能不稳定；建议使用至少 15 个非共面点。"
                : string.Empty;
            if (pictures.Length > 3)
            {
                warning += (warning.Length > 0 ? "\n" : string.Empty) +
                           $"已启用 {pictures.Length - 3} 个附加稀疏机位；" +
                           "它们只使用各自实际标出的基础点和参考线。";
                int[] minimumPointCameras = Enumerable
                    .Range(3, pictures.Length - 3)
                    .Where(cameraIndex =>
                        pointMaps[cameraIndex].Keys.Count(basePointIdSet.Contains) < 6)
                    .ToArray();
                if (minimumPointCameras.Length > 0)
                {
                    warning += "\n附加 Camera " + string.Join(", ", minimumPointCameras) +
                               " 只有 4～5 个可见点，正在使用最小解模式；" +
                               "请尽量让点分散，并在条件允许时增加到 6 个以上。";
                }
                if (additionalPointIds.Length > 0)
                {
                    warning += $"\n将从附加机位共同观测中生成 {additionalPointIds.Length} 个新增三维点；" +
                               "这些点不计入附加相机初始化所需的 4 个基础点。";
                }
                int[] poseOnlyCameras = Enumerable
                    .Range(3, pictures.Length - 3)
                    .Where(cameraIndex => pictures[cameraIndex].CameraPoseOnly)
                    .ToArray();
                if (poseOnlyCameras.Length > 0)
                {
                    warning += "\nCamera " + string.Join(", ", poseOnlyCameras) +
                               " 仅求相机位姿：公共三维结构固定后，" +
                               "它们的参考点和参考线只用于计算各自的位置、旋转和 FOV。";
                }
            }
            input = new ReconstructionInput(
                pictures,
                cameras,
                pointIds,
                basePointIds.Length,
                observations,
                observationVisibility,
                observationConfidences,
                lineIds,
                lineObservations,
                lineObservationVisibility,
                resultXAxisLineId,
                options,
                warning);
            return true;
        }

        internal static Vector2 WorldToTopLeftPixel(RectTransform picture, Vector3 worldPosition)
        {
            Vector3 local = picture.InverseTransformPoint(worldPosition);
            Rect rect = picture.rect;
            return new Vector2(local.x - rect.xMin, rect.yMax - local.y);
        }

        /// <summary> 判断二维无限直线是否穿过图片矩形。 </summary>
        private static bool LineIntersectsPicture(
            Vector2 start,
            Vector2 end,
            int width,
            int height)
        {
            float a = start.y - end.y;
            float b = end.x - start.x;
            float c = start.x * end.y - end.x * start.y;
            float value0 = c;
            float value1 = a * width + c;
            float value2 = b * height + c;
            float value3 = a * width + b * height + c;
            float minimum = Mathf.Min(value0, value1, value2, value3);
            float maximum = Mathf.Max(value0, value1, value2, value3);
            return minimum <= 0f && maximum >= 0f;
        }

        private static bool TryGetPictureSize(
            RefPicture picture,
            out int width,
            out int height,
            out string error)
        {
            Rect rect = picture.RectTransform.rect;
            width = Mathf.RoundToInt(rect.width);
            height = Mathf.RoundToInt(rect.height);
            if (width <= 0 || height <= 0 ||
                Mathf.Abs(rect.width - width) > 0.001f || Mathf.Abs(rect.height - height) > 0.001f)
            {
                error = $"Camera {picture.CameraId} 的 RectTransform 尺寸必须是正整数像素，" +
                        $"当前为 {rect.width:F3}×{rect.height:F3}。";
                return false;
            }

            error = string.Empty;
            return true;
        }
    }
}
