using UnityEngine;

namespace ReconstructionTool
{
    [DisallowMultipleComponent]
    public sealed class RefPoint3D : MonoBehaviour
    {
        [SerializeField] private int id;
        [SerializeField] private float reprojectionRms;
        [SerializeField] private float gizmoRadius = 0.015f;

        public int Id => id;
        public float ReprojectionRms => reprojectionRms;

        public void Initialize(int newId, float error)
        {
            id = newId;
            reprojectionRms = error;
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(transform.position, Mathf.Max(0.001f, gizmoRadius));
        }
    }
}
