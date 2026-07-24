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
        private SerializedProperty scalePointIdA;
        private SerializedProperty scalePointIdB;
        private SerializedProperty scaleReferenceDistance;

        private void OnEnable()
        {
            cameraId = serializedObject.FindProperty("cameraId");
            minimumVerticalFov = serializedObject.FindProperty("minimumVerticalFov");
            maximumVerticalFov = serializedObject.FindProperty("maximumVerticalFov");
            scalePointIdA = serializedObject.FindProperty("scalePointIdA");
            scalePointIdB = serializedObject.FindProperty("scalePointIdB");
            scaleReferenceDistance = serializedObject.FindProperty("scaleReferenceDistance");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();
            EditorGUILayout.PropertyField(cameraId, new GUIContent("Camera ID"));
            EditorGUILayout.PropertyField(minimumVerticalFov, new GUIContent("最小垂直 FOV"));
            EditorGUILayout.PropertyField(maximumVerticalFov, new GUIContent("最大垂直 FOV"));

            if (cameraId.intValue == 0)
            {
                EditorGUILayout.Space();
                EditorGUILayout.LabelField("绝对尺度（Camera 0）", EditorStyles.boldLabel);
                EditorGUILayout.PropertyField(scalePointIdA, new GUIContent("参考点 ID A"));
                EditorGUILayout.PropertyField(scalePointIdB, new GUIContent("参考点 ID B"));
                EditorGUILayout.PropertyField(scaleReferenceDistance, new GUIContent("真实距离"));
            }

            serializedObject.ApplyModifiedProperties();
            EditorGUILayout.Space();
            EditorGUILayout.HelpBox(
                "RectTransform 宽高必须等于原图像素尺寸；图片须按原比例显示，不能非等比拉伸或做未知裁剪。",
                MessageType.Info);

            if (GUILayout.Button("Solve", GUILayout.Height(32f)))
            {
                ReconstructionSceneSolver.Solve();
            }
        }
    }
}
