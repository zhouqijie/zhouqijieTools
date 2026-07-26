using UnityEngine;

namespace ReconstructionTool
{
    [DisallowMultipleComponent]
    public sealed class RefPointConnection3D : MonoBehaviour
    {
        [SerializeField] private RefPoint3D pointA;
        [SerializeField] private RefPoint3D pointB;

        public RefPoint3D PointA => pointA;
        public RefPoint3D PointB => pointB;

        /// <summary> 使用两个三维参考点初始化独立连线。 </summary>
        public void Initialize(RefPoint3D newPointA, RefPoint3D newPointB)
        {
            pointA = newPointA;
            pointB = newPointB;
        }
    }
}
