using UnityEngine;

namespace ReconstructionTool
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(RectTransform))]
    public sealed class RefLine2D : MonoBehaviour
    {
        [SerializeField] private int id;
        [SerializeField, Range(0.1f, 1f)]
        [Tooltip("与所属 RefPicture 的图片置信度相乘，作为该线观测的求解权重。")]
        private float confidence = 1f;
        [SerializeField, InspectorName("作为结果 X 轴")]
        [Tooltip("将这个 Line ID 对应的三维直线对齐到世界 X 轴。相同 Line ID 只需勾选一个。")]
        private bool useAsResultXAxis;

        public int Id => id;
        public float Confidence => confidence;
        public bool UseAsResultXAxis => useAsResultXAxis;
        public RectTransform RectTransform => (RectTransform)transform;

        /// <summary> 返回仅用于定义二维无限直线的两个操作端点。 </summary>
        public void GetWorldEndpoints(out Vector3 start, out Vector3 end)
        {
            Rect rect = RectTransform.rect;
            float centerY = rect.center.y;
            start = RectTransform.TransformPoint(new Vector3(rect.xMin, centerY, 0f));
            end = RectTransform.TransformPoint(new Vector3(rect.xMax, centerY, 0f));
        }

        private void OnValidate()
        {
            confidence = Mathf.Clamp(confidence, 0.1f, 1f);
        }
    }
}
