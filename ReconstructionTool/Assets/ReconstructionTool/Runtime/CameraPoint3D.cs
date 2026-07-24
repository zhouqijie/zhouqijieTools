using UnityEngine;

namespace ReconstructionTool
{
    [DisallowMultipleComponent]
    public sealed class CameraPoint3D : MonoBehaviour
    {
        [SerializeField] private int cameraId;
        [SerializeField] private RefPicture sourcePicture;
        [SerializeField] private float focalLengthPixels;
        [SerializeField] private float horizontalFov;
        [SerializeField] private float verticalFov;
        [SerializeField] private float reprojectionRms;
        [SerializeField] private float gizmoDepth = 0.5f;

        public int CameraId => cameraId;
        public RefPicture SourcePicture => sourcePicture;
        public float FocalLengthPixels => focalLengthPixels;
        public float HorizontalFov => horizontalFov;
        public float VerticalFov => verticalFov;
        public float ReprojectionRms => reprojectionRms;

        public void Initialize(
            int newCameraId,
            RefPicture picture,
            float focalPixels,
            float horizontalFovDegrees,
            float verticalFovDegrees,
            float error)
        {
            cameraId = newCameraId;
            sourcePicture = picture;
            focalLengthPixels = focalPixels;
            horizontalFov = horizontalFovDegrees;
            verticalFov = verticalFovDegrees;
            reprojectionRms = error;
        }

        private void OnDrawGizmos()
        {
            float depth = Mathf.Max(0.01f, gizmoDepth);
            float halfHeight = Mathf.Tan(verticalFov * Mathf.Deg2Rad * 0.5f) * depth;
            float halfWidth = Mathf.Tan(horizontalFov * Mathf.Deg2Rad * 0.5f) * depth;
            Vector3 p0 = new(-halfWidth, -halfHeight, depth);
            Vector3 p1 = new(halfWidth, -halfHeight, depth);
            Vector3 p2 = new(halfWidth, halfHeight, depth);
            Vector3 p3 = new(-halfWidth, halfHeight, depth);

            Gizmos.color = Color.cyan;
            Gizmos.matrix = transform.localToWorldMatrix;
            Gizmos.DrawLine(Vector3.zero, p0);
            Gizmos.DrawLine(Vector3.zero, p1);
            Gizmos.DrawLine(Vector3.zero, p2);
            Gizmos.DrawLine(Vector3.zero, p3);
            Gizmos.DrawLine(p0, p1);
            Gizmos.DrawLine(p1, p2);
            Gizmos.DrawLine(p2, p3);
            Gizmos.DrawLine(p3, p0);
        }
    }
}
