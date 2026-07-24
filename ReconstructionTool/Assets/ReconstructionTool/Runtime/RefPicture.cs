using UnityEngine;

namespace ReconstructionTool
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Canvas), typeof(RectTransform))]
    public sealed class RefPicture : MonoBehaviour
    {
        [SerializeField, Range(0, 2)] private int cameraId;
        [SerializeField, Range(1f, 178f)] private float minimumVerticalFov = 15f;
        [SerializeField, Range(1f, 178f)] private float maximumVerticalFov = 120f;
        [SerializeField] private int scalePointIdA;
        [SerializeField] private int scalePointIdB = 1;
        [SerializeField, Min(0.000001f)] private float scaleReferenceDistance = 1f;

        public int CameraId => cameraId;
        public float MinimumVerticalFov => minimumVerticalFov;
        public float MaximumVerticalFov => maximumVerticalFov;
        public int ScalePointIdA => scalePointIdA;
        public int ScalePointIdB => scalePointIdB;
        public float ScaleReferenceDistance => scaleReferenceDistance;
        public RectTransform RectTransform => (RectTransform)transform;

        private void OnValidate()
        {
            cameraId = Mathf.Clamp(cameraId, 0, 2);
            minimumVerticalFov = Mathf.Clamp(minimumVerticalFov, 1f, 178f);
            maximumVerticalFov = Mathf.Clamp(maximumVerticalFov, minimumVerticalFov + 0.01f, 179f);
            scaleReferenceDistance = Mathf.Max(scaleReferenceDistance, 0.000001f);
        }
    }
}
