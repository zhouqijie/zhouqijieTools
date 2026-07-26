using UnityEngine;

namespace ReconstructionTool
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(RectTransform))]
    public sealed class RefPoint2D : MonoBehaviour
    {
        [SerializeField] private int id;
        [SerializeField, Range(0.1f, 1f)]
        [Tooltip("与所属 RefPicture 的图片置信度相乘，作为该点观测的求解权重。")]
        private float confidence = 1f;

        public int Id => id;
        public float Confidence => confidence;
        public RectTransform RectTransform => (RectTransform)transform;

        private void OnValidate()
        {
            confidence = Mathf.Clamp(confidence, 0.1f, 1f);
        }
    }
}
