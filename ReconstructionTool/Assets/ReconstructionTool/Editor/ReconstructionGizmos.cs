using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace ReconstructionTool.Editor
{
    internal static class ReconstructionGizmos
    {
        [DrawGizmo(GizmoType.Selected | GizmoType.NonSelected)]
        private static void DrawCameraLabel(CameraPoint3D camera, GizmoType gizmoType)
        {
            Handles.Label(camera.transform.position, $"Camera {camera.CameraId}");
        }

        /// <summary> 在 SceneView 中绘制二维参考点 ID。 </summary>
        [DrawGizmo(GizmoType.Selected | GizmoType.NonSelected)]
        private static void DrawPointLabel(RefPoint2D point, GizmoType gizmoType)
        {
            Handles.Label(point.transform.position, $"ID {point.Id}");
        }

        [DrawGizmo(GizmoType.Selected | GizmoType.NonSelected)]
        private static void DrawPointLabel(RefPoint3D point, GizmoType gizmoType)
        {
            Handles.Label(point.transform.position, $"ID {point.Id}");
        }

        /// <summary> 绘制二维参考线的操作段、整条图片内直线和 ID。 </summary>
        [DrawGizmo(GizmoType.Selected | GizmoType.NonSelected)]
        private static void DrawLine2D(RefLine2D line, GizmoType gizmoType)
        {
            line.GetWorldEndpoints(out Vector3 handleStart, out Vector3 handleEnd);
            Color oldColor = Handles.color;
            Handles.color = new Color(1f, 0.2f, 0.85f, 0.95f);
            Handles.DrawAAPolyLine(4f, handleStart, handleEnd);

            RefPicture picture = line.GetComponentInParent<RefPicture>();
            if (picture != null &&
                TryGetClippedLine(
                    picture.RectTransform,
                    handleStart,
                    handleEnd,
                    out Vector3 clippedStart,
                    out Vector3 clippedEnd))
            {
                Handles.DrawDottedLine(clippedStart, clippedEnd, 5f);
            }

            string axisLabel = line.UseAsResultXAxis ? "  [X Axis]" : string.Empty;
            Handles.Label((handleStart + handleEnd) * 0.5f, $"Line {line.Id}{axisLabel}");
            Handles.color = oldColor;
        }

        /// <summary> 绘制求解得到的三维无限直线及其误差。 </summary>
        [DrawGizmo(GizmoType.Selected | GizmoType.NonSelected)]
        private static void DrawLine3D(RefLine3D line, GizmoType gizmoType)
        {
            Vector3 halfLine = line.transform.TransformVector(
                Vector3.forward * (line.GizmoLength * 0.5f));
            Color oldColor = Handles.color;
            Handles.color = new Color(1f, 0.35f, 0.05f, 0.95f);
            Handles.DrawAAPolyLine(
                4f,
                line.transform.position - halfLine,
                line.transform.position + halfLine);
            Handles.Label(
                line.transform.position,
                $"Line {line.Id}  RMS {line.ReprojectionRms:F2}px");
            Handles.color = oldColor;
        }

        /// <summary> 按 Camera 0 定义的 ID 关系，在每张图片内绘制二维连线。 </summary>
        [DrawGizmo(GizmoType.Selected | GizmoType.NonSelected)]
        private static void DrawPictureConnections(RefPicture picture, GizmoType gizmoType)
        {
            RefPicture definition = picture.CameraId == 0 ? picture : null;
            if (definition == null)
            {
                RefPicture[] pictures = Object.FindObjectsByType<RefPicture>(
                    FindObjectsInactive.Include,
                    FindObjectsSortMode.None);
                for (int index = 0; index < pictures.Length; index++)
                {
                    if (pictures[index].CameraId == 0 &&
                        pictures[index].gameObject.scene == picture.gameObject.scene)
                    {
                        definition = pictures[index];
                        break;
                    }
                }
            }

            IReadOnlyList<RefPointConnection> connections = definition?.PointConnections;
            if (connections == null || connections.Count == 0)
            {
                return;
            }

            var pointsById = new Dictionary<int, RefPoint2D>();
            RefPoint2D[] points = picture.GetComponentsInChildren<RefPoint2D>(true);
            for (int index = 0; index < points.Length; index++)
            {
                pointsById.TryAdd(points[index].Id, points[index]);
            }

            Color oldColor = Handles.color;
            Handles.color = new Color(0f, 1f, 1f, 0.9f);
            for (int index = 0; index < connections.Count; index++)
            {
                RefPointConnection connection = connections[index];
                if (connection.PointIdA != connection.PointIdB &&
                    pointsById.TryGetValue(connection.PointIdA, out RefPoint2D start) &&
                    pointsById.TryGetValue(connection.PointIdB, out RefPoint2D end))
                {
                    Handles.DrawAAPolyLine(3f, start.transform.position, end.transform.position);
                }
            }
            Handles.color = oldColor;
        }

        /// <summary> 使用独立连接组件持有的端点绘制三维连线。 </summary>
        [DrawGizmo(GizmoType.Selected | GizmoType.NonSelected)]
        private static void DrawPointConnection3D(
            RefPointConnection3D connection,
            GizmoType gizmoType)
        {
            if (connection.PointA == null || connection.PointB == null)
            {
                return;
            }

            Color oldColor = Handles.color;
            Handles.color = new Color(1f, 0.65f, 0f, 0.95f);
            Handles.DrawAAPolyLine(
                3f,
                connection.PointA.transform.position,
                connection.PointB.transform.position);
            Handles.color = oldColor;
        }

        /// <summary> 将二维无限直线裁剪到参考图片矩形内。 </summary>
        private static bool TryGetClippedLine(
            RectTransform picture,
            Vector3 worldStart,
            Vector3 worldEnd,
            out Vector3 clippedStart,
            out Vector3 clippedEnd)
        {
            Vector2 start = picture.InverseTransformPoint(worldStart);
            Vector2 end = picture.InverseTransformPoint(worldEnd);
            Vector2 direction = end - start;
            Rect rect = picture.rect;
            var intersections = new Vector2[4];
            int count = 0;

            if (Mathf.Abs(direction.x) > 0.000001f)
            {
                float t = (rect.xMin - start.x) / direction.x;
                float y = start.y + t * direction.y;
                if (y >= rect.yMin - 0.001f && y <= rect.yMax + 0.001f)
                {
                    intersections[count++] = new Vector2(rect.xMin, y);
                }

                t = (rect.xMax - start.x) / direction.x;
                y = start.y + t * direction.y;
                if (y >= rect.yMin - 0.001f && y <= rect.yMax + 0.001f)
                {
                    intersections[count++] = new Vector2(rect.xMax, y);
                }
            }

            if (Mathf.Abs(direction.y) > 0.000001f)
            {
                float t = (rect.yMin - start.y) / direction.y;
                float x = start.x + t * direction.x;
                if (x >= rect.xMin - 0.001f && x <= rect.xMax + 0.001f)
                {
                    intersections[count++] = new Vector2(x, rect.yMin);
                }

                t = (rect.yMax - start.y) / direction.y;
                x = start.x + t * direction.x;
                if (x >= rect.xMin - 0.001f && x <= rect.xMax + 0.001f)
                {
                    intersections[count++] = new Vector2(x, rect.yMax);
                }
            }

            if (count < 2)
            {
                clippedStart = default;
                clippedEnd = default;
                return false;
            }

            int bestFirst = 0;
            int bestSecond = 1;
            float bestDistance = 0f;
            for (int first = 0; first < count; first++)
            {
                for (int second = first + 1; second < count; second++)
                {
                    float distance = (intersections[first] - intersections[second]).sqrMagnitude;
                    if (distance > bestDistance)
                    {
                        bestFirst = first;
                        bestSecond = second;
                        bestDistance = distance;
                    }
                }
            }

            clippedStart = picture.TransformPoint(intersections[bestFirst]);
            clippedEnd = picture.TransformPoint(intersections[bestSecond]);
            return bestDistance > 0.000001f;
        }
    }
}
