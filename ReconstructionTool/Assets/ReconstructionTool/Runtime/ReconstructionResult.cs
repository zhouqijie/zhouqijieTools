using UnityEngine;

namespace ReconstructionTool
{
    [DisallowMultipleComponent]
    public sealed class ReconstructionResult : MonoBehaviour
    {
        [SerializeField] private float reprojectionRms;
        [SerializeField] private float scaleMultiplier;
        [SerializeField] private float medianTriangulationAngle;
        [SerializeField, TextArea] private string solveInformation;

        public float ReprojectionRms => reprojectionRms;
        public float ScaleMultiplier => scaleMultiplier;
        public float MedianTriangulationAngle => medianTriangulationAngle;
        public string SolveInformation => solveInformation;

        public void Initialize(float rms, float scale, float medianAngle, string information)
        {
            reprojectionRms = rms;
            scaleMultiplier = scale;
            medianTriangulationAngle = medianAngle;
            solveInformation = information;
        }
    }
}
