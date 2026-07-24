using NUnit.Framework;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace ReconstructionTool.Editor.Tests
{
    internal sealed class ReconstructionInputCollectorTests
    {
        [SetUp]
        public void SetUp()
        {
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        }

        [TearDown]
        public void TearDown()
        {
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        }

        [Test]
        public void WorldToTopLeftPixel_HandlesPivotScaleAndRotation()
        {
            GameObject pictureObject = new("Picture", typeof(RectTransform));
            RectTransform picture = pictureObject.GetComponent<RectTransform>();
            picture.pivot = new Vector2(0.17f, 0.81f);
            picture.sizeDelta = new Vector2(640f, 480f);
            picture.position = new Vector3(11f, -7f, 3f);
            picture.rotation = Quaternion.Euler(13f, -21f, 37f);
            picture.localScale = new Vector3(1.7f, 0.65f, 1f);

            Vector2 expected = new(123.5f, 312.25f);
            Vector3 local = new(
                picture.rect.xMin + expected.x,
                picture.rect.yMax - expected.y,
                0f);
            Vector3 world = picture.TransformPoint(local);

            Vector2 actual = ReconstructionInputCollector.WorldToTopLeftPixel(picture, world);

            Assert.That(actual.x, Is.EqualTo(expected.x).Within(0.001f));
            Assert.That(actual.y, Is.EqualTo(expected.y).Within(0.001f));
        }

        [Test]
        public void TryCollect_AcceptsDifferentImageSizesAndPivots()
        {
            CreateValidSetup(8);

            bool success = ReconstructionInputCollector.TryCollect(out ReconstructionInput input, out string error);

            Assert.That(success, Is.True, error);
            Assert.That(input.Cameras[0].Width, Is.EqualTo(1200));
            Assert.That(input.Cameras[1].Width, Is.EqualTo(1920));
            Assert.That(input.Cameras[2].Height, Is.EqualTo(1600));
            Assert.That(input.PointIds, Has.Length.EqualTo(8));
            Assert.That(input.Observations, Has.Length.EqualTo(24));
            Assert.That(input.Warning, Is.Not.Empty);
        }

        [Test]
        public void TryCollect_RejectsDuplicateCameraId()
        {
            RefPicture[] pictures = CreateValidSetup(8);
            SetProperty(pictures[2], "cameraId", 1);

            bool success = ReconstructionInputCollector.TryCollect(out _, out string error);

            Assert.That(success, Is.False);
            StringAssert.Contains("0、1、2", error);
        }

        [Test]
        public void TryCollect_RejectsDuplicatePointId()
        {
            RefPicture[] pictures = CreateValidSetup(8);
            AddPoint(pictures[1], 0, new Vector2(300f, 300f));

            bool success = ReconstructionInputCollector.TryCollect(out _, out string error);

            Assert.That(success, Is.False);
            StringAssert.Contains("重复", error);
        }

        [Test]
        public void TryCollect_RejectsMismatchedPointSet()
        {
            RefPicture[] pictures = CreateValidSetup(8);
            Object.DestroyImmediate(pictures[2].GetComponentsInChildren<RefPoint2D>()[7].gameObject);

            bool success = ReconstructionInputCollector.TryCollect(out _, out string error);

            Assert.That(success, Is.False);
            StringAssert.Contains("不一致", error);
        }

        [Test]
        public void TryCollect_RejectsPointOutsidePicture()
        {
            RefPicture[] pictures = CreateValidSetup(8);
            RefPoint2D point = pictures[0].GetComponentsInChildren<RefPoint2D>()[0];
            Rect rect = pictures[0].RectTransform.rect;
            point.RectTransform.localPosition = new Vector3(rect.xMax + 20f, rect.yMax + 20f, 0f);

            bool success = ReconstructionInputCollector.TryCollect(out _, out string error);

            Assert.That(success, Is.False);
            StringAssert.Contains("图片矩形外", error);
        }

        [Test]
        public void TryCollect_RejectsFewerThanEightPoints()
        {
            CreateValidSetup(7);

            bool success = ReconstructionInputCollector.TryCollect(out _, out string error);

            Assert.That(success, Is.False);
            StringAssert.Contains("至少需要 8", error);
        }

        [Test]
        public void NativePlugin_ReportsPinnedVersion()
        {
            StringAssert.StartsWith("ReconstructionNative/", ReconstructionNativeApi.Version);
        }

        [Test]
        public void Solve_CreatesScaledHierarchyAndSecondSolveIsUndoable()
        {
            (RefPicture[] pictures, Vector3[] points, float knownDistance) = CreateSyntheticSetup();

            ReconstructionSceneSolver.Solve();

            ReconstructionResult firstResult = Object.FindFirstObjectByType<ReconstructionResult>();
            Assert.That(firstResult, Is.Not.Null);
            Transform cameraRoot = firstResult.transform.Find("Cameras");
            Transform pointRoot = firstResult.transform.Find("Points");
            Assert.That(cameraRoot, Is.Not.Null);
            Assert.That(pointRoot, Is.Not.Null);
            Assert.That(cameraRoot.childCount, Is.EqualTo(3));
            Assert.That(pointRoot.childCount, Is.EqualTo(points.Length));
            Assert.That(cameraRoot.GetChild(0).position.sqrMagnitude, Is.LessThan(0.000001f));
            Assert.That(
                Quaternion.Angle(cameraRoot.GetChild(0).rotation, Quaternion.identity),
                Is.LessThan(0.01f));

            RefPoint3D pointA = FindPoint(pointRoot, 100);
            RefPoint3D pointB = FindPoint(pointRoot, 129);
            Assert.That(
                Vector3.Distance(pointA.transform.position, pointB.transform.position),
                Is.EqualTo(knownDistance).Within(0.001f));
            Assert.That(firstResult.ReprojectionRms, Is.LessThan(0.1f));
            int firstResultInstanceId = firstResult.GetInstanceID();

            ReconstructionSceneSolver.Solve();
            ReconstructionResult secondResult = Object.FindFirstObjectByType<ReconstructionResult>();
            Assert.That(secondResult, Is.Not.Null);
            Assert.That(secondResult, Is.Not.SameAs(firstResult));
            Assert.That(Object.FindObjectsByType<ReconstructionResult>(
                FindObjectsInactive.Include, FindObjectsSortMode.None), Has.Length.EqualTo(1));

            Undo.PerformUndo();
            ReconstructionResult restoredResult = Object.FindFirstObjectByType<ReconstructionResult>();
            Assert.That(restoredResult.GetInstanceID(), Is.EqualTo(firstResultInstanceId));
            Assert.That(Object.FindObjectsByType<ReconstructionResult>(
                FindObjectsInactive.Include, FindObjectsSortMode.None), Has.Length.EqualTo(1));
            Assert.That(pictures[0], Is.Not.Null);
        }

        private static RefPicture[] CreateValidSetup(int pointCount)
        {
            Vector2[] sizes =
            {
                new(1200f, 900f),
                new(1920f, 1080f),
                new(900f, 1600f)
            };
            Vector2[] pivots =
            {
                new(0.5f, 0.5f),
                new(0f, 1f),
                new(0.23f, 0.71f)
            };
            var pictures = new RefPicture[3];
            for (int cameraIndex = 0; cameraIndex < 3; cameraIndex++)
            {
                GameObject pictureObject = new(
                    $"Picture_{cameraIndex}",
                    typeof(RectTransform),
                    typeof(Canvas),
                    typeof(RefPicture));
                RefPicture picture = pictureObject.GetComponent<RefPicture>();
                picture.RectTransform.sizeDelta = sizes[cameraIndex];
                picture.RectTransform.pivot = pivots[cameraIndex];
                picture.RectTransform.position = new Vector3(cameraIndex * 2500f, cameraIndex * -300f, 0f);
                picture.RectTransform.rotation = Quaternion.Euler(0f, 0f, cameraIndex * 13f);
                picture.RectTransform.localScale = Vector3.one * (1f + cameraIndex * 0.2f);
                SetProperty(picture, "cameraId", cameraIndex);
                pictures[cameraIndex] = picture;

                for (int pointIndex = 0; pointIndex < pointCount; pointIndex++)
                {
                    float x = 80f + pointIndex * (sizes[cameraIndex].x - 160f) /
                        Mathf.Max(1, pointCount - 1);
                    float y = 100f + (pointIndex % 4) * (sizes[cameraIndex].y - 200f) / 3f;
                    AddPoint(picture, pointIndex, new Vector2(x, y));
                }
            }

            SetProperty(pictures[0], "scalePointIdA", 0);
            SetProperty(pictures[0], "scalePointIdB", 1);
            SetProperty(pictures[0], "scaleReferenceDistance", 2.5f);
            return pictures;
        }

        private static (RefPicture[] pictures, Vector3[] points, float knownDistance) CreateSyntheticSetup()
        {
            Vector2Int[] sizes =
            {
                new(4000, 3000),
                new(1920, 1080),
                new(3024, 4032)
            };
            float[] fovs = { 52f, 68f, 44f };
            Vector3[] centers =
            {
                Vector3.zero,
                new(1.15f, 0.08f, 0.18f),
                new(-0.8f, -0.12f, 0.35f)
            };
            var points = new Vector3[30];
            for (int index = 0; index < points.Length; index++)
            {
                points[index] = new Vector3(
                    Mathf.Sin(index * 1.71f) * 0.78f,
                    Mathf.Cos(index * 1.13f) * 0.52f,
                    4.1f + (index * 17 % 29) / 29f * 2.8f);
            }

            var pictures = new RefPicture[3];
            for (int cameraIndex = 0; cameraIndex < 3; cameraIndex++)
            {
                GameObject pictureObject = new(
                    $"SyntheticPicture_{cameraIndex}",
                    typeof(RectTransform),
                    typeof(Canvas),
                    typeof(RefPicture));
                RefPicture picture = pictureObject.GetComponent<RefPicture>();
                picture.RectTransform.sizeDelta = sizes[cameraIndex];
                picture.RectTransform.position = new Vector3(cameraIndex * 5000f, 0f, 0f);
                SetProperty(picture, "cameraId", cameraIndex);
                pictures[cameraIndex] = picture;

                float focal = sizes[cameraIndex].y /
                    (2f * Mathf.Tan(fovs[cameraIndex] * Mathf.Deg2Rad * 0.5f));
                for (int pointIndex = 0; pointIndex < points.Length; pointIndex++)
                {
                    Vector3 cameraPoint = points[pointIndex] - centers[cameraIndex];
                    cameraPoint = cameraIndex switch
                    {
                        1 => RotateY(cameraPoint, 0.18f),
                        2 => RotateY(RotateX(cameraPoint, 0.04f), -0.15f),
                        _ => cameraPoint
                    };
                    Vector2 pixel = new(
                        focal * cameraPoint.x / cameraPoint.z + sizes[cameraIndex].x * 0.5f,
                        focal * cameraPoint.y / cameraPoint.z + sizes[cameraIndex].y * 0.5f);
                    AddPoint(picture, 100 + pointIndex, pixel);
                }
            }

            float knownDistance = Vector3.Distance(points[0], points[^1]);
            SetProperty(pictures[0], "scalePointIdA", 100);
            SetProperty(pictures[0], "scalePointIdB", 129);
            SetProperty(pictures[0], "scaleReferenceDistance", knownDistance);
            return (pictures, points, knownDistance);
        }

        private static Vector3 RotateX(Vector3 value, float radians)
        {
            float cosine = Mathf.Cos(radians);
            float sine = Mathf.Sin(radians);
            return new Vector3(
                value.x,
                cosine * value.y - sine * value.z,
                sine * value.y + cosine * value.z);
        }

        private static Vector3 RotateY(Vector3 value, float radians)
        {
            float cosine = Mathf.Cos(radians);
            float sine = Mathf.Sin(radians);
            return new Vector3(
                cosine * value.x + sine * value.z,
                value.y,
                -sine * value.x + cosine * value.z);
        }

        private static RefPoint3D FindPoint(Transform pointRoot, int id)
        {
            foreach (RefPoint3D point in pointRoot.GetComponentsInChildren<RefPoint3D>())
            {
                if (point.Id == id)
                {
                    return point;
                }
            }
            Assert.Fail($"Did not find RefPoint3D ID {id}.");
            return null;
        }

        private static void AddPoint(RefPicture picture, int id, Vector2 pixel)
        {
            GameObject pointObject = new($"Point_{id}", typeof(RectTransform), typeof(RefPoint2D));
            RectTransform point = pointObject.GetComponent<RectTransform>();
            point.SetParent(picture.transform, false);
            Rect rect = picture.RectTransform.rect;
            point.localPosition = new Vector3(rect.xMin + pixel.x, rect.yMax - pixel.y, 0f);
            SetProperty(pointObject.GetComponent<RefPoint2D>(), "id", id);
        }

        private static void SetProperty(Object target, string name, int value)
        {
            var serializedObject = new SerializedObject(target);
            serializedObject.FindProperty(name).intValue = value;
            serializedObject.ApplyModifiedPropertiesWithoutUndo();
        }

        private static void SetProperty(Object target, string name, float value)
        {
            var serializedObject = new SerializedObject(target);
            serializedObject.FindProperty(name).floatValue = value;
            serializedObject.ApplyModifiedPropertiesWithoutUndo();
        }
    }
}
