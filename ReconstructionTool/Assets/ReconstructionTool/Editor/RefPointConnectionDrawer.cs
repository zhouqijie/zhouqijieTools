using UnityEditor;
using UnityEngine;

namespace ReconstructionTool.Editor
{
    [CustomPropertyDrawer(typeof(RefPointConnection))]
    internal sealed class RefPointConnectionDrawer : PropertyDrawer
    {
        /// <summary> 将一条连线的起点和终点 ID 显示在同一行。 </summary>
        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            EditorGUI.BeginProperty(position, label, property);
            Rect content = EditorGUI.PrefixLabel(position, label);
            float gap = 6f;
            float width = (content.width - gap) * 0.5f;
            Rect startRect = new(content.x, content.y, width, content.height);
            Rect endRect = new(content.x + width + gap, content.y, width, content.height);

            float oldLabelWidth = EditorGUIUtility.labelWidth;
            EditorGUIUtility.labelWidth = 32f;
            EditorGUI.PropertyField(
                startRect,
                property.FindPropertyRelative("pointIdA"),
                new GUIContent("起点"));
            EditorGUI.PropertyField(
                endRect,
                property.FindPropertyRelative("pointIdB"),
                new GUIContent("终点"));
            EditorGUIUtility.labelWidth = oldLabelWidth;
            EditorGUI.EndProperty();
        }
    }
}
