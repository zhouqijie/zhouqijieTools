using UnityEngine;

namespace ReconstructionTool
{
    [DisallowMultipleComponent]
    public sealed class RefLine3D : MonoBehaviour
    {
        [SerializeField] private int id;
        [SerializeField] private float reprojectionRms;
        [SerializeField, Min(0.001f)] private float gizmoLength = 1f;

        public int Id => id;
        public Vector3 Direction => transform.forward;
        public float ReprojectionRms => reprojectionRms;
        public float GizmoLength => gizmoLength;

        /// <summary> 初始化求解得到的三维无限直线。 </summary>
        public void Initialize(int newId, Vector3 newDirection, float error, float length)
        {
            id = newId;
            SetWorldDirection(newDirection);
            reprojectionRms = error;
            gizmoLength = Mathf.Max(0.001f, length);
        }

        /// <summary> 使用 Transform 的世界旋转表达直线方向。 </summary>
        private void SetWorldDirection(Vector3 worldDirection)
        {
            Vector3 direction = worldDirection.sqrMagnitude > 0.000001f
                ? worldDirection.normalized
                : Vector3.forward;
            Vector3 up = Mathf.Abs(Vector3.Dot(direction, Vector3.up)) > 0.999f
                ? Vector3.right
                : Vector3.up;
            transform.rotation = Quaternion.LookRotation(direction, up);
        }
    }
}
