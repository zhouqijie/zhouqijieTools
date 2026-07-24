using UnityEngine;

namespace ReconstructionTool
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(RectTransform))]
    public sealed class RefPoint2D : MonoBehaviour
    {
        [SerializeField] private int id;

        public int Id => id;
        public RectTransform RectTransform => (RectTransform)transform;
    }
}
