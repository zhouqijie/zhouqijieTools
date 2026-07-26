using UnityEditor;
using UnityEngine;

namespace ReconstructionTool.Editor
{
    [CustomEditor(typeof(RefPicture))]
    internal sealed class RefPictureEditor : UnityEditor.Editor
    {
        private SerializedProperty cameraId;
        private SerializedProperty minimumVerticalFov;
        private SerializedProperty maximumVerticalFov;
        private SerializedProperty confidence;
        private SerializedProperty cameraPoseOnly;
        private SerializedProperty pointConnections;
        private SerializedProperty scalePointIdA;
        private SerializedProperty scalePointIdB;
        private SerializedProperty scaleReferenceDistance;
        private SerializedProperty maximumNormalizedReprojectionError;

        private void OnEnable()
        {
            cameraId = serializedObject.FindProperty("cameraId");
            minimumVerticalFov = serializedObject.FindProperty("minimumVerticalFov");
            maximumVerticalFov = serializedObject.FindProperty("maximumVerticalFov");
            confidence = serializedObject.FindProperty("confidence");
            cameraPoseOnly = serializedObject.FindProperty("cameraPoseOnly");
            pointConnections = serializedObject.FindProperty("pointConnections");
            scalePointIdA = serializedObject.FindProperty("scalePointIdA");
            scalePointIdB = serializedObject.FindProperty("scalePointIdB");
            scaleReferenceDistance = serializedObject.FindProperty("scaleReferenceDistance");
            maximumNormalizedReprojectionError =
                serializedObject.FindProperty("maximumNormalizedReprojectionError");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();
            EditorGUILayout.PropertyField(cameraId, new GUIContent("Camera ID"));
            EditorGUILayout.PropertyField(minimumVerticalFov, new GUIContent("最小垂直 FOV"));
            EditorGUILayout.PropertyField(maximumVerticalFov, new GUIContent("最大垂直 FOV"));
            EditorGUILayout.PropertyField(
                confidence,
                new GUIContent(
                    "图片置信度",
                    "整个机位的相对权重；会分别与每个 RefPoint2D/RefLine2D 的置信度相乘。"));
            EditorGUILayout.PropertyField(
                cameraPoseOnly,
                new GUIContent(
                    "仅求相机位姿",
                    "仅允许 Camera 3+：不参与公共点/线重建，固定已有三维结构后单独求此机位。"));
            if (cameraPoseOnly.boolValue)
            {
                EditorGUILayout.HelpBox(
                    cameraId.intValue < 3
                        ? "Camera 0～2 是基础三视图，不能设为仅求相机位姿。"
                        : "该机位不会改变公共 RefPoint3D 或 RefLine3D；其点和线只用于计算本机位的位置、旋转和 FOV。",
                    cameraId.intValue < 3 ? MessageType.Error : MessageType.Info);
            }

            if (cameraId.intValue == 0)
            {
                EditorGUILayout.Space();
                EditorGUILayout.LabelField("参考点连线（Camera 0）", EditorStyles.boldLabel);
                EditorGUILayout.PropertyField(
                    pointConnections,
                    new GUIContent(
                        "连线关系",
                        "只需在 Camera 0 定义。每组 ID 会绘制到三张图片，并复制到三维重建结果。"),
                    true);

                EditorGUILayout.Space();
                EditorGUILayout.LabelField("绝对尺度（Camera 0）", EditorStyles.boldLabel);
                EditorGUILayout.PropertyField(scalePointIdA, new GUIContent("参考点 ID A"));
                EditorGUILayout.PropertyField(scalePointIdB, new GUIContent("参考点 ID B"));
                EditorGUILayout.PropertyField(scaleReferenceDistance, new GUIContent("真实距离"));

                EditorGUILayout.Space();
                EditorGUILayout.LabelField("求解质量（Camera 0）", EditorStyles.boldLabel);
                EditorGUILayout.PropertyField(
                    maximumNormalizedReprojectionError,
                    new GUIContent(
                        "最大重投影容差",
                        "图片长边归一化为 1000 像素后的 RMS 容差；3 相当于长边的 0.3%，" +
                        "原图容差=3×原图长边/1000。换算为米还取决于物距和焦距。"));
            }

            serializedObject.ApplyModifiedProperties();
            EditorGUILayout.Space();
            EditorGUILayout.HelpBox(
                "RectTransform 宽高比必须与完整原图一致，图片不能有未知裁剪、留白或非等比拉伸；" +
                "若希望日志中的原图像素误差准确，宽高应使用原图像素尺寸。",
                MessageType.Info);
            EditorGUILayout.HelpBox(
                "Camera 0～2 是基础三视图，必须标出相同的至少 8 个点；Camera 3 及以后是附加机位，" +
                "每个机位仍需至少 4 个可见基础点，建议 6 个以上；附加机位之间还可以用相同新 ID " +
                "共同标记基础三视图看不到的新点。仅求相机位姿的机位不能用于生成这些新点。" +
                "所有 Camera ID 必须从 0 连续编号。",
                MessageType.Info);
            EditorGUILayout.HelpBox(
                "置信度只会降低该图片对解算的影响，不能校正镜头畸变。建议至少保留一张高质量图片为 1；" +
                "最大重投影容差超过 5 时，结果应仅作为低精度参考。",
                MessageType.Info);

            EditorGUILayout.Space();
            EditorGUILayout.HelpBox(
                "参考线用于对齐图片里能看清、但端点不明确的直边。同一物理直边需要在三张图片中使用相同 Line ID；" +
                "操作段端点不要求对应，位置和旋转只用于定义整条无限直线。" +
                "附加机位仍需至少 4 个基础点完成 P4Pf 初始定位，随后参考线会参与细化本机位。",
                MessageType.Info);
            if (GUILayout.Button("创建 RefLine2D", GUILayout.Height(24f)))
            {
                CreateReferenceLine((RefPicture)target);
            }

            if (GUILayout.Button("Solve", GUILayout.Height(32f)))
            {
                ReconstructionSceneSolver.Solve();
            }
        }

        /// <summary> 在当前图片中央创建一条可移动、旋转和缩放的参考线操作段。 </summary>
        private static void CreateReferenceLine(RefPicture picture)
        {
            int nextId = 0;
            foreach (RefLine2D existing in picture.GetComponentsInChildren<RefLine2D>(true))
            {
                nextId = Mathf.Max(nextId, existing.Id + 1);
            }

            GameObject lineObject = new("RefLine2D", typeof(RectTransform));
            Undo.RegisterCreatedObjectUndo(lineObject, "Create RefLine2D");
            Undo.SetTransformParent(lineObject.transform, picture.transform, "Create RefLine2D");
            RectTransform lineTransform = lineObject.GetComponent<RectTransform>();
            lineTransform.anchorMin = new Vector2(0.5f, 0.5f);
            lineTransform.anchorMax = new Vector2(0.5f, 0.5f);
            lineTransform.pivot = new Vector2(0.5f, 0.5f);
            lineTransform.anchoredPosition = Vector2.zero;
            lineTransform.localRotation = Quaternion.identity;
            lineTransform.localScale = Vector3.one;
            lineTransform.sizeDelta = new Vector2(
                Mathf.Max(100f, picture.RectTransform.rect.width * 0.3f),
                8f);

            RefLine2D line = Undo.AddComponent<RefLine2D>(lineObject);
            var lineSerialized = new SerializedObject(line);
            lineSerialized.FindProperty("id").intValue = nextId;
            lineSerialized.ApplyModifiedPropertiesWithoutUndo();
            Selection.activeGameObject = lineObject;
            EditorGUIUtility.PingObject(lineObject);
        }
    }
}
