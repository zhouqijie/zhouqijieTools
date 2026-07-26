using System;
using UnityEngine;

namespace ReconstructionTool
{
    [Serializable]
    public struct RefPointConnection
    {
        [SerializeField, InspectorName("起点 ID")] private int pointIdA;
        [SerializeField, InspectorName("终点 ID")] private int pointIdB;

        public int PointIdA => pointIdA;
        public int PointIdB => pointIdB;

        /// <summary> 创建一条由两个参考点 ID 定义的连线。 </summary>
        public RefPointConnection(int newPointIdA, int newPointIdB)
        {
            pointIdA = newPointIdA;
            pointIdB = newPointIdB;
        }
    }
}
