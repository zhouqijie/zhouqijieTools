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

        [DrawGizmo(GizmoType.Selected)]
        private static void DrawPointLabel(RefPoint3D point, GizmoType gizmoType)
        {
            Handles.Label(point.transform.position, $"ID {point.Id}");
        }
    }
}
