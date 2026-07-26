using UnityEngine;
using UnityEngine.UI;

namespace ReconstructionTool
{
    [ExecuteAlways]
    [DisallowMultipleComponent]
    [RequireComponent(typeof(RawImage))]
    public sealed class RawImageLensDistortionCorrector : MonoBehaviour, IMaterialModifier
    {
        private const string ShaderResourceName = "RawImageLensDistortionCorrection";

        private static readonly int RadialK1Id = Shader.PropertyToID("_RadialK1");
        private static readonly int RadialK2Id = Shader.PropertyToID("_RadialK2");
        private static readonly int DistortionCenterId = Shader.PropertyToID("_DistortionCenter");
        private static readonly int CorrectionZoomId = Shader.PropertyToID("_CorrectionZoom");
        private static readonly int UvRectId = Shader.PropertyToID("_UvRect");

        [SerializeField, Range(-1f, 1f), InspectorName("一次径向系数 K1")]
        [Tooltip("原图为桶形畸变时使用负值，枕形畸变时使用正值。建议从 ±0.05 开始调整。")]
        private float radialK1;

        [SerializeField, Range(-1f, 1f), InspectorName("二次径向系数 K2")]
        [Tooltip("用于修正边缘方向反转的胡须形/波浪形畸变。普通桶形修正通常保持为 0。")]
        private float radialK2;

        [SerializeField, InspectorName("畸变中心")]
        [Tooltip("镜头光学中心在图片中的归一化位置，通常为 (0.5, 0.5)。")]
        private Vector2 distortionCenter = new(0.5f, 0.5f);

        [SerializeField, Range(0.5f, 2f), InspectorName("修正后缩放")]
        [Tooltip("大于 1 会放大画面，可裁掉修正后出现的透明边缘。")]
        private float correctionZoom = 1f;

        private RawImage rawImage;
        private Material correctionMaterial;

        public float RadialK1 => radialK1;
        public float RadialK2 => radialK2;
        public Vector2 DistortionCenter => distortionCenter;
        public float CorrectionZoom => correctionZoom;

        /// <summary> 为 RawImage 返回支持 UI 遮罩的畸变修正材质。 </summary>
        public Material GetModifiedMaterial(Material baseMaterial)
        {
            EnsureMaterial();
            if (correctionMaterial == null)
            {
                return baseMaterial;
            }

            CopyUiMaterialState(baseMaterial);
            ApplyProperties();
            return correctionMaterial;
        }

        private void OnEnable()
        {
            rawImage = GetComponent<RawImage>();
            EnsureMaterial();
            ApplyProperties();
            rawImage.SetMaterialDirty();
        }

        private void OnDisable()
        {
            if (rawImage != null)
            {
                rawImage.SetMaterialDirty();
            }
            DestroyMaterial();
        }

        private void OnValidate()
        {
            distortionCenter.x = Mathf.Clamp01(distortionCenter.x);
            distortionCenter.y = Mathf.Clamp01(distortionCenter.y);
            correctionZoom = Mathf.Clamp(correctionZoom, 0.5f, 2f);
            if (!isActiveAndEnabled)
            {
                return;
            }

            rawImage = GetComponent<RawImage>();
            EnsureMaterial();
            ApplyProperties();
            rawImage.SetMaterialDirty();
        }

        private void LateUpdate()
        {
            ApplyProperties();
        }

        /// <summary> 创建仅供当前 RawImage 使用的运行时材质。 </summary>
        private void EnsureMaterial()
        {
            if (correctionMaterial != null)
            {
                return;
            }

            Shader shader = Resources.Load<Shader>(ShaderResourceName);
            if (shader == null)
            {
                Debug.LogError(
                    $"[ReconstructionTool] 找不到 Resources/{ShaderResourceName}.shader。",
                    this);
                return;
            }

            correctionMaterial = new Material(shader)
            {
                name = $"{name} Lens Distortion Correction",
                hideFlags = HideFlags.HideAndDontSave
            };
        }

        /// <summary> 更新径向修正参数和 RawImage 的 UV 区域。 </summary>
        private void ApplyProperties()
        {
            if (correctionMaterial == null || rawImage == null)
            {
                return;
            }

            Rect uvRect = rawImage.uvRect;
            correctionMaterial.SetFloat(RadialK1Id, radialK1);
            correctionMaterial.SetFloat(RadialK2Id, radialK2);
            correctionMaterial.SetVector(DistortionCenterId, distortionCenter);
            correctionMaterial.SetFloat(CorrectionZoomId, correctionZoom);
            correctionMaterial.SetVector(
                UvRectId,
                new Vector4(uvRect.x, uvRect.y, uvRect.width, uvRect.height));
        }

        /// <summary> 保留 Mask、RectMask2D 和 UI Alpha Clip 的材质状态。 </summary>
        private void CopyUiMaterialState(Material baseMaterial)
        {
            if (baseMaterial == null || correctionMaterial == null)
            {
                return;
            }

            string[] propertyNames =
            {
                "_Stencil",
                "_StencilComp",
                "_StencilOp",
                "_StencilWriteMask",
                "_StencilReadMask",
                "_ColorMask",
                "_UseUIAlphaClip"
            };
            foreach (string propertyName in propertyNames)
            {
                if (baseMaterial.HasProperty(propertyName) &&
                    correctionMaterial.HasProperty(propertyName))
                {
                    correctionMaterial.SetFloat(
                        propertyName,
                        baseMaterial.GetFloat(propertyName));
                }
            }

            correctionMaterial.shaderKeywords = baseMaterial.shaderKeywords;
        }

        /// <summary> 销毁编辑态或运行态创建的临时材质。 </summary>
        private void DestroyMaterial()
        {
            if (correctionMaterial == null)
            {
                return;
            }

            if (Application.isPlaying)
            {
                Destroy(correctionMaterial);
            }
            else
            {
                DestroyImmediate(correctionMaterial);
            }
            correctionMaterial = null;
        }
    }
}
