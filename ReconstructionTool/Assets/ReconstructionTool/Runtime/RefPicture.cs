using System.Collections.Generic;
using UnityEngine;

namespace ReconstructionTool
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Canvas), typeof(RectTransform))]
    public sealed class RefPicture : MonoBehaviour
    {
        [SerializeField, Min(0)] private int cameraId;
        [SerializeField, Range(1f, 178f)] private float minimumVerticalFov = 15f;
        [SerializeField, Range(1f, 178f)] private float maximumVerticalFov = 120f;
        [SerializeField, Range(0.1f, 1f)] private float confidence = 1f;
        [SerializeField, Tooltip("仅允许 Camera 3+ 使用。固定公共三维点和参考线，只求此机位的位置、旋转和 FOV。")]
        private bool cameraPoseOnly;
        [SerializeField] private List<RefPointConnection> pointConnections = new();
        [SerializeField] private int scalePointIdA;
        [SerializeField] private int scalePointIdB = 1;
        [SerializeField, Min(0.000001f)] private float scaleReferenceDistance = 1f;
        [SerializeField, Min(0.1f)] private float maximumNormalizedReprojectionError = 1.5f;

        public int CameraId => cameraId;
        public float MinimumVerticalFov => minimumVerticalFov;
        public float MaximumVerticalFov => maximumVerticalFov;
        public float Confidence => confidence;
        public bool CameraPoseOnly => cameraPoseOnly;
        public IReadOnlyList<RefPointConnection> PointConnections => pointConnections;
        public int ScalePointIdA => scalePointIdA;
        public int ScalePointIdB => scalePointIdB;
        public float ScaleReferenceDistance => scaleReferenceDistance;
        public float MaximumNormalizedReprojectionError => maximumNormalizedReprojectionError;
        public RectTransform RectTransform => (RectTransform)transform;

        private void OnValidate()
        {
            cameraId = Mathf.Max(0, cameraId);
            minimumVerticalFov = Mathf.Clamp(minimumVerticalFov, 1f, 178f);
            maximumVerticalFov = Mathf.Clamp(maximumVerticalFov, minimumVerticalFov + 0.01f, 179f);
            confidence = Mathf.Clamp(confidence, 0.1f, 1f);
            scaleReferenceDistance = Mathf.Max(scaleReferenceDistance, 0.000001f);
            maximumNormalizedReprojectionError =
                Mathf.Max(maximumNormalizedReprojectionError, 0.1f);
        }
    }
}
