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
        internal ReconstructionNativeApi.Observation[] Observations { get; }
        internal ReconstructionNativeApi.SolveOptions Options { get; }
        internal string Warning { get; }

        internal ReconstructionInput(
            RefPicture[] pictures,
            ReconstructionNativeApi.CameraInput[] cameras,
            int[] pointIds,
            ReconstructionNativeApi.Observation[] observations,
            ReconstructionNativeApi.SolveOptions options,
            string warning)
        {
            Pictures = pictures;
            Cameras = cameras;
            PointIds = pointIds;
            Observations = observations;
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
            if (pictures.Length != 3)
            {
                error = $"场景中必须恰好有 3 个 RefPicture，当前找到 {pictures.Length} 个。";
                return false;
            }

            if (pictures.Select(picture => picture.CameraId).Distinct().Count() != 3 ||
                pictures.Any(picture => picture.CameraId is < 0 or > 2))
            {
                error = "三个 RefPicture 的 CameraId 必须唯一且正好为 0、1、2。";
                return false;
            }

            Array.Sort(pictures, (left, right) => left.CameraId.CompareTo(right.CameraId));
            var pointMaps = new Dictionary<int, RefPoint2D>[3];
            for (int cameraIndex = 0; cameraIndex < 3; cameraIndex++)
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
            }

            int[] pointIds = pointMaps[0].Keys.OrderBy(id => id).ToArray();
            if (pointIds.Length < 8)
            {
                error = $"至少需要 8 个共同参考点，当前只有 {pointIds.Length} 个。";
                return false;
            }

            for (int cameraIndex = 1; cameraIndex < 3; cameraIndex++)
            {
                if (!pointIds.SequenceEqual(pointMaps[cameraIndex].Keys.OrderBy(id => id)))
                {
                    error = $"Camera {cameraIndex} 的 RefPoint2D ID 集合与 Camera 0 不一致。";
                    return false;
                }
            }

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

            var cameras = new ReconstructionNativeApi.CameraInput[3];
            var observations = new ReconstructionNativeApi.Observation[3 * pointIds.Length];
            for (int cameraIndex = 0; cameraIndex < 3; cameraIndex++)
            {
                RefPicture picture = pictures[cameraIndex];
                TryGetPictureSize(picture, out int width, out int height, out _);
                cameras[cameraIndex] = new ReconstructionNativeApi.CameraInput
                {
                    Width = width,
                    Height = height,
                    MinimumVerticalFov = picture.MinimumVerticalFov,
                    MaximumVerticalFov = picture.MaximumVerticalFov
                };

                for (int pointIndex = 0; pointIndex < pointIds.Length; pointIndex++)
                {
                    RefPoint2D point = pointMaps[cameraIndex][pointIds[pointIndex]];
                    Vector2 pixel = WorldToTopLeftPixel(picture.RectTransform, point.transform.position);
                    observations[cameraIndex * pointIds.Length + pointIndex] =
                        new ReconstructionNativeApi.Observation { X = pixel.x, Y = pixel.y };
                }
            }

            var options = new ReconstructionNativeApi.SolveOptions
            {
                ScalePointIdA = scalePicture.ScalePointIdA,
                ScalePointIdB = scalePicture.ScalePointIdB,
                KnownScaleDistance = scalePicture.ScaleReferenceDistance,
                MaximumNormalizedReprojectionError = 1.5,
                RandomSeed = 1337,
                MaximumCandidates = 12
            };
            string warning = pointIds.Length < 15
                ? $"只有 {pointIds.Length} 个共同点，结果可能不稳定；建议使用至少 15 个非共面点。"
                : string.Empty;
            input = new ReconstructionInput(pictures, cameras, pointIds, observations, options, warning);
            return true;
        }

        internal static Vector2 WorldToTopLeftPixel(RectTransform picture, Vector3 worldPosition)
        {
            Vector3 local = picture.InverseTransformPoint(worldPosition);
            Rect rect = picture.rect;
            return new Vector2(local.x - rect.xMin, rect.yMax - local.y);
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
