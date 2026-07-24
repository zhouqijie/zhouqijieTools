using System;
using System.Linq;
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
                EditorUtility.DisplayDialog("三视图重建失败", error, "确定");
                return;
            }

            if (!string.IsNullOrEmpty(input.Warning))
            {
                Debug.LogWarning($"[ReconstructionTool] {input.Warning}");
            }

            ReconstructionNativeApi.CameraOutput[] cameras;
            ReconstructionNativeApi.PointOutput[] points;
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
                    input.Observations,
                    input.Options,
                    out cameras,
                    out points,
                    out report,
                    out nativeError);
            }
            catch (DllNotFoundException exception)
            {
                EditorUtility.DisplayDialog(
                    "缺少原生插件",
                    "未找到 ReconstructionNative.dll。请先运行 Native/ReconstructionNative/Build-Native.ps1。\n\n" +
                    exception.Message,
                    "确定");
                return;
            }
            catch (Exception exception)
            {
                EditorUtility.DisplayDialog("原生求解器调用失败", exception.Message, "确定");
                return;
            }

            if (status != ReconstructionNativeApi.Status.Success)
            {
                string details = string.IsNullOrWhiteSpace(nativeError) ? "未提供详细原因。" : nativeError;
                EditorUtility.DisplayDialog($"三视图重建失败（{status}）", details, "确定");
                return;
            }

            Undo.IncrementCurrentGroup();
            int undoGroup = Undo.GetCurrentGroup();
            Undo.SetCurrentGroupName("Solve Three-View Reconstruction");
            bool completed = false;
            try
            {
                ReconstructionResult[] oldResults = UnityEngine.Object
                    .FindObjectsByType<ReconstructionResult>(
                        FindObjectsInactive.Include,
                        FindObjectsSortMode.None)
                    .Where(result => result.gameObject.scene.IsValid())
                    .ToArray();

                GameObject root = new("ReconstructionResult");
                Undo.RegisterCreatedObjectUndo(root, "Create Reconstruction Result");
                Scene targetScene = input.Pictures[0].gameObject.scene;
                if (targetScene.IsValid() && root.scene != targetScene)
                {
                    SceneManager.MoveGameObjectToScene(root, targetScene);
                }

                ReconstructionResult result = Undo.AddComponent<ReconstructionResult>(root);
                result.Initialize(
                    (float)report.NormalizedReprojectionRms,
                    (float)report.AppliedScale,
                    (float)report.MedianTriangulationAngle,
                    $"Native {nativeVersion}; {report.InlierCount}/{report.PointCount} inliers");
                EditorUtility.SetDirty(result);

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
                }

                foreach (ReconstructionResult oldResult in oldResults)
                {
                    Undo.DestroyObjectImmediate(oldResult.gameObject);
                }

                Selection.activeGameObject = root;
                EditorGUIUtility.PingObject(root);
                Debug.Log(
                    $"[ReconstructionTool] 求解成功：{points.Length} 点，" +
                    $"RMS={report.NormalizedReprojectionRms:F3}px，" +
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
    }
}
