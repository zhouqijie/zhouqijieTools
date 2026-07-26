using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace ReconstructionTool.Editor
{
    internal static class ReconstructionSceneSolver
    {
        internal static void Solve()
        {
            if (!ReconstructionInputCollector.TryCollect(out ReconstructionInput input, out string error))
            {
                EditorUtility.DisplayDialog("多视图重建失败", error, "确定");
                return;
            }

            if (!string.IsNullOrEmpty(input.Warning))
            {
                Debug.LogWarning($"[ReconstructionTool] {input.Warning}");
            }

            ReconstructionNativeApi.CameraOutput[] cameras;
            ReconstructionNativeApi.PointOutput[] points;
            ReconstructionNativeApi.LineOutput[] lines;
            ReconstructionNativeApi.SolveReport report;
            string nativeError;
            ReconstructionNativeApi.Status status;
            string nativeVersion;
            try
            {
                nativeVersion = ReconstructionNativeApi.Version;
                status = ReconstructionNativeApi.Solve(
                    input.Cameras,
                    input.PointIds,
                    input.BasePointCount,
                    input.Observations,
                    input.ObservationVisibility,
                    input.ObservationConfidences,
                    input.LineIds,
                    input.LineObservations,
                    input.LineObservationVisibility,
                    input.Options,
                    out cameras,
                    out points,
                    out lines,
                    out report,
                    out nativeError);
            }
            catch (DllNotFoundException exception)
            {
                EditorUtility.DisplayDialog(
                    "缺少原生插件",
                    "未找到 ReconstructionNative_1_8.dll。请先运行 Native/ReconstructionNative/Build-Native.ps1。\n\n" +
                    exception.Message,
                    "确定");
                return;
            }
            catch (Exception exception)
            {
                EditorUtility.DisplayDialog("原生求解器调用失败", exception.Message, "确定");
                return;
            }

            string diagnostics = ReconstructionCameraDiagnostics.Build(
                input,
                cameras,
                points,
                out bool hasDiagnosticWarning);
            if (!string.IsNullOrEmpty(diagnostics))
            {
                if (status != ReconstructionNativeApi.Status.Success || hasDiagnosticWarning)
                {
                    Debug.LogWarning(diagnostics);
                }
                else
                {
                    Debug.Log(diagnostics);
                }
            }

            if (status != ReconstructionNativeApi.Status.Success)
            {
                string details = string.IsNullOrWhiteSpace(nativeError)
                    ? "未提供详细原因。"
                    : FormatNativeError(nativeError, input.Pictures);
                Debug.LogError(
                    $"[ReconstructionTool] 多视图重建失败（{status}）：\n{details}");
                EditorUtility.DisplayDialog($"多视图重建失败（{status}）", details, "确定");
                return;
            }

            Undo.IncrementCurrentGroup();
            int undoGroup = Undo.GetCurrentGroup();
            Undo.SetCurrentGroupName("Solve Multi-View Reconstruction");
            bool completed = false;
            try
            {
                GameObject[] oldResults = UnityEngine.Object
                    .FindObjectsByType<CameraPoint3D>(
                        FindObjectsInactive.Include,
                        FindObjectsSortMode.None)
                    .Where(camera => camera.gameObject.scene.IsValid())
                    .Select(camera => camera.transform.root.gameObject)
                    .Where(candidate => candidate.name == "ReconstructionResult")
                    .Distinct()
                    .ToArray();

                GameObject root = new("ReconstructionResult");
                Undo.RegisterCreatedObjectUndo(root, "Create Reconstruction Result");
                Scene targetScene = input.Pictures[0].gameObject.scene;
                if (targetScene.IsValid() && root.scene != targetScene)
                {
                    SceneManager.MoveGameObjectToScene(root, targetScene);
                }

                GameObject cameraRoot = CreateChild(root.transform, "Cameras");
                for (int cameraIndex = 0; cameraIndex < cameras.Length; cameraIndex++)
                {
                    ReconstructionNativeApi.CameraOutput camera = cameras[cameraIndex];
                    GameObject cameraObject = CreateChild(cameraRoot.transform, $"CameraPoint_{cameraIndex}");
                    cameraObject.transform.SetPositionAndRotation(
                        new Vector3(
                            (float)camera.PositionX,
                            (float)camera.PositionY,
                            (float)camera.PositionZ),
                        new Quaternion(
                            (float)camera.RotationX,
                            (float)camera.RotationY,
                            (float)camera.RotationZ,
                            (float)camera.RotationW));
                    CameraPoint3D component = Undo.AddComponent<CameraPoint3D>(cameraObject);
                    component.Initialize(
                        cameraIndex,
                        input.Pictures[cameraIndex],
                        (float)camera.FocalLengthPixels,
                        (float)camera.HorizontalFov,
                        (float)camera.VerticalFov,
                        (float)camera.ReprojectionRmsPixels);
                    EditorUtility.SetDirty(component);
                }

                GameObject pointRoot = CreateChild(root.transform, "Points");
                var pointsById = new Dictionary<int, RefPoint3D>();
                foreach (ReconstructionNativeApi.PointOutput point in points.OrderBy(point => point.Id))
                {
                    GameObject pointObject = CreateChild(pointRoot.transform, $"RefPoint_{point.Id}");
                    pointObject.transform.position = new Vector3(
                        (float)point.PositionX,
                        (float)point.PositionY,
                        (float)point.PositionZ);
                    RefPoint3D component = Undo.AddComponent<RefPoint3D>(pointObject);
                    component.Initialize(point.Id, (float)point.ReprojectionRmsPixels);
                    EditorUtility.SetDirty(component);
                    pointsById.Add(point.Id, component);
                }

                if (input.Pictures[0].PointConnections.Count > 0)
                {
                    GameObject connectionRoot = CreateChild(root.transform, "Connections");
                    foreach (RefPointConnection connection in input.Pictures[0].PointConnections)
                    {
                        if (connection.PointIdA == connection.PointIdB ||
                            !pointsById.TryGetValue(connection.PointIdA, out RefPoint3D pointA) ||
                            !pointsById.TryGetValue(connection.PointIdB, out RefPoint3D pointB))
                        {
                            continue;
                        }

                        GameObject connectionObject = CreateChild(
                            connectionRoot.transform,
                            $"Connection_{connection.PointIdA}_{connection.PointIdB}");
                        RefPointConnection3D component =
                            Undo.AddComponent<RefPointConnection3D>(connectionObject);
                        component.Initialize(pointA, pointB);
                        EditorUtility.SetDirty(component);
                    }
                }

                if (lines.Length > 0)
                {
                    Bounds pointBounds = new(
                        new Vector3(
                            (float)points[0].PositionX,
                            (float)points[0].PositionY,
                            (float)points[0].PositionZ),
                        Vector3.zero);
                    for (int pointIndex = 1; pointIndex < points.Length; pointIndex++)
                    {
                        pointBounds.Encapsulate(new Vector3(
                            (float)points[pointIndex].PositionX,
                            (float)points[pointIndex].PositionY,
                            (float)points[pointIndex].PositionZ));
                    }

                    float gizmoLength = Mathf.Max(0.1f, pointBounds.size.magnitude);
                    GameObject lineRoot = CreateChild(root.transform, "Lines");
                    foreach (ReconstructionNativeApi.LineOutput line in lines.OrderBy(line => line.Id))
                    {
                        GameObject lineObject = CreateChild(lineRoot.transform, $"RefLine_{line.Id}");
                        lineObject.transform.position = new Vector3(
                            (float)line.PointX,
                            (float)line.PointY,
                            (float)line.PointZ);
                        RefLine3D component = Undo.AddComponent<RefLine3D>(lineObject);
                        component.Initialize(
                            line.Id,
                            new Vector3(
                                (float)line.DirectionX,
                                (float)line.DirectionY,
                                (float)line.DirectionZ),
                            (float)line.ReprojectionRmsPixels,
                            gizmoLength);
                        EditorUtility.SetDirty(component);
                    }
                }

                // 按无向参考线将整个结果对齐到最接近的世界 X 轴方向。
                if (input.ResultXAxisLineId is int resultXAxisLineId)
                {
                    ReconstructionNativeApi.LineOutput axisLine =
                        lines.First(line => line.Id == resultXAxisLineId);
                    Vector3 direction = new(
                        (float)axisLine.DirectionX,
                        (float)axisLine.DirectionY,
                        (float)axisLine.DirectionZ);
                    if (!(direction.sqrMagnitude > 0.000001f))
                    {
                        throw new InvalidOperationException(
                            $"参考线 {resultXAxisLineId} 的三维方向无效，无法对齐 X 轴。");
                    }

                    direction.Normalize();
                    Vector3 targetDirection = Vector3.Dot(direction, Vector3.right) >= 0f
                        ? Vector3.right
                        : Vector3.left;
                    root.transform.rotation =
                        Quaternion.FromToRotation(direction, targetDirection);
                    Debug.Log(
                        $"[ReconstructionTool] 已将参考线 {resultXAxisLineId} " +
                        "对齐到世界 X 轴方向。");
                }

                foreach (GameObject oldResult in oldResults)
                {
                    Undo.DestroyObjectImmediate(oldResult);
                }

                Selection.activeGameObject = root;
                EditorGUIUtility.PingObject(root);
                Debug.Log(
                    $"[ReconstructionTool] 求解成功：{points.Length} 点、{lines.Length} 条参考线，" +
                    $"原生版本={nativeVersion}，" +
                    $"RMS={report.NormalizedReprojectionRms:F3}px，" +
                    $"线RMS={report.NormalizedLineRms:F3}px，" +
                    $"中值三角化角={report.MedianTriangulationAngle:F2}°，" +
                    $"尺度倍率={report.AppliedScale:F6}。");
                completed = true;
            }
            catch (Exception exception)
            {
                Undo.RevertAllDownToGroup(undoGroup);
                EditorUtility.DisplayDialog(
                    "生成重建结果失败",
                    $"求解已经完成，但无法安全写入场景；旧结果已保留。\n\n{exception.Message}",
                    "确定");
            }
            finally
            {
                if (completed)
                {
                    Undo.CollapseUndoOperations(undoGroup);
                }
            }
        }

        private static GameObject CreateChild(Transform parent, string name)
        {
            GameObject child = new(name);
            Undo.RegisterCreatedObjectUndo(child, "Create Reconstruction Result");
            Undo.SetTransformParent(child.transform, parent, "Create Reconstruction Result");
            child.transform.SetLocalPositionAndRotation(Vector3.zero, Quaternion.identity);
            return child;
        }

        /// <summary> 本地化原生错误，并将 Camera 编号补充为 Canvas 名称。 </summary>
        private static string FormatNativeError(
            string message,
            IReadOnlyList<RefPicture> pictures)
        {
            string localized = message
                .Replace(
                    "These line IDs have excessive reprojection error:",
                    "以下参考线的重投影误差超过容差：")
                .Replace(
                    "These pose-only cameras have excessive reprojection error:",
                    "以下“仅求相机位姿”机位的重投影误差超过容差：")
                .Replace(
                    "Parallel lines are supported. Adjust the listed Canvas RefLine2D; " +
                    "the projected 3D line is too far from that marked 2D line.",
                    "允许使用平行参考线。请调整上面明确列出的 Canvas 中对应的 RefLine2D；" +
                    "当前三维线投影后没有贴合这条二维标线。")
                .Replace("combined RMS=", "综合 RMS=")
                .Replace("normalized px", "归一化像素")
                .Replace("source-image px", "原图像素")
                .Replace("allowed=", "容差=")
                .Replace("confidence=", "组合置信度=");
            return Regex.Replace(
                localized,
                @"\bCamera (?<index>\d+)\b",
                match =>
                {
                    if (!int.TryParse(match.Groups["index"].Value, out int cameraIndex) ||
                        cameraIndex < 0 ||
                        cameraIndex >= pictures.Count ||
                        pictures[cameraIndex] == null)
                    {
                        return match.Value;
                    }

                    return $"{match.Value}「{pictures[cameraIndex].gameObject.name}」";
                });
        }
    }
}
