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
            Assert.That(input.BasePointCount, Is.EqualTo(8));
            Assert.That(input.Observations, Has.Length.EqualTo(24));
            Assert.That(input.Warning, Is.Not.Empty);
        }

        /// <summary> 验证附加机位只需提供基础点集中的一部分。 </summary>
        [Test]
        public void TryCollect_AcceptsSparseAdditionalCamera()
        {
            CreateValidSetup(8);
            RefPicture additional = CreateAdditionalPicture(3);
            for (int pointId = 0; pointId < 4; pointId++)
            {
                AddPoint(
                    additional,
                    pointId,
                    new Vector2(120f + pointId * 180f, 160f + pointId % 3 * 220f));
            }

            bool success = ReconstructionInputCollector.TryCollect(
                out ReconstructionInput input,
                out string error);

            Assert.That(success, Is.True, error);
            Assert.That(input.Pictures, Has.Length.EqualTo(4));
            Assert.That(input.Cameras, Has.Length.EqualTo(4));
            Assert.That(input.Observations, Has.Length.EqualTo(32));
            Assert.That(input.ObservationVisibility, Has.Length.EqualTo(32));
            for (int pointIndex = 0; pointIndex < 8; pointIndex++)
            {
                Assert.That(
                    input.ObservationVisibility[3 * 8 + pointIndex],
                    Is.EqualTo(pointIndex < 4 ? 1 : 0));
            }
            StringAssert.Contains("1 个附加稀疏机位", input.Warning);
            StringAssert.Contains("最小解模式", input.Warning);
        }

        /// <summary> 验证附加机位可以只求自身位姿而不参与公共结构重建。 </summary>
        [Test]
        public void TryCollect_AcceptsPoseOnlyAdditionalCamera()
        {
            CreateValidSetup(8);
            RefPicture additional = CreateAdditionalPicture(3);
            SetProperty(additional, "cameraPoseOnly", true);
            for (int pointId = 0; pointId < 4; pointId++)
            {
                AddPoint(additional, pointId, new Vector2(100f + pointId * 150f, 200f));
            }

            bool success = ReconstructionInputCollector.TryCollect(
                out ReconstructionInput input,
                out string error);

            Assert.That(success, Is.True, error);
            Assert.That(input.Cameras[3].PoseOnly, Is.EqualTo(1));
            StringAssert.Contains("仅求相机位姿", input.Warning);
        }

        /// <summary> 验证基础三视图不能设为仅求相机位姿。 </summary>
        [Test]
        public void TryCollect_RejectsPoseOnlyBaseCamera()
        {
            RefPicture[] pictures = CreateValidSetup(8);
            SetProperty(pictures[2], "cameraPoseOnly", true);

            bool success = ReconstructionInputCollector.TryCollect(out _, out string error);

            Assert.That(success, Is.False);
            StringAssert.Contains("Camera 0～2", error);
        }

        /// <summary> 验证仅定位机位不能单独声明尚未生成的三维点。 </summary>
        [Test]
        public void TryCollect_RejectsUnknownPointOnPoseOnlyCamera()
        {
            CreateValidSetup(8);
            RefPicture additional = CreateAdditionalPicture(3);
            SetProperty(additional, "cameraPoseOnly", true);
            for (int pointId = 0; pointId < 4; pointId++)
            {
                AddPoint(additional, pointId, new Vector2(100f + pointId * 150f, 200f));
            }
            AddPoint(additional, 99, new Vector2(800f, 500f));

            bool success = ReconstructionInputCollector.TryCollect(out _, out string error);

            Assert.That(success, Is.False);
            StringAssert.Contains("尚未由参与公共重建的机位生成", error);
        }

        /// <summary> 验证两个附加机位可以共同生成基础三视图不可见的新点。 </summary>
        [Test]
        public void TryCollect_AcceptsSharedAdditionalPoint()
        {
            CreateValidSetup(8);
            RefPicture additional3 = CreateAdditionalPicture(3);
            RefPicture additional4 = CreateAdditionalPicture(4);
            for (int pointId = 0; pointId < 4; pointId++)
            {
                AddPoint(additional3, pointId, new Vector2(100f + pointId * 150f, 200f));
                AddPoint(additional4, pointId + 4, new Vector2(120f + pointId * 150f, 400f));
            }
            AddPoint(additional3, 99, new Vector2(500f, 500f));
            AddPoint(additional4, 99, new Vector2(700f, 450f));

            bool success = ReconstructionInputCollector.TryCollect(
                out ReconstructionInput input,
                out string error);

            Assert.That(success, Is.True, error);
            Assert.That(input.BasePointCount, Is.EqualTo(8));
            Assert.That(input.PointIds, Is.EqualTo(new[] { 0, 1, 2, 3, 4, 5, 6, 7, 99 }));
            Assert.That(input.ObservationVisibility[3 * 9 + 8], Is.EqualTo(1));
            Assert.That(input.ObservationVisibility[4 * 9 + 8], Is.EqualTo(1));
            StringAssert.Contains("生成 1 个新增三维点", input.Warning);
        }

        /// <summary> 验证附加新点不能只有一个机位观测。 </summary>
        [Test]
        public void TryCollect_RejectsAdditionalPointSeenByOnlyOneCamera()
        {
            CreateValidSetup(8);
            RefPicture additional3 = CreateAdditionalPicture(3);
            RefPicture additional4 = CreateAdditionalPicture(4);
            for (int pointId = 0; pointId < 4; pointId++)
            {
                AddPoint(additional3, pointId, new Vector2(100f + pointId * 150f, 200f));
                AddPoint(additional4, pointId + 4, new Vector2(120f + pointId * 150f, 400f));
            }
            AddPoint(additional3, 99, new Vector2(500f, 500f));

            bool success = ReconstructionInputCollector.TryCollect(out _, out string error);

            Assert.That(success, Is.False);
            StringAssert.Contains("至少需要被两个附加机位", error);
        }

        /// <summary> 验证附加机位至少需要四个可见基础点。 </summary>
        [Test]
        public void TryCollect_RejectsTooFewAdditionalPoints()
        {
            CreateValidSetup(8);
            RefPicture additional = CreateAdditionalPicture(3);
            for (int pointId = 0; pointId < 3; pointId++)
            {
                AddPoint(additional, pointId, new Vector2(100f + pointId * 150f, 200f));
            }

            bool success = ReconstructionInputCollector.TryCollect(out _, out string error);

            Assert.That(success, Is.False);
            StringAssert.Contains("至少需要标出 4 个", error);
        }

        /// <summary> 验证图片、逐点置信度和全局容差会进入原生求解输入。 </summary>
        [Test]
        public void TryCollect_UsesConfidenceAndConfiguredTolerance()
        {
            RefPicture[] pictures = CreateValidSetup(8);
            SetProperty(pictures[0], "confidence", 0.35f);
            SetProperty(pictures[1], "confidence", 0.6f);
            SetProperty(pictures[2], "confidence", 0.9f);
            RefPoint2D uncertainPoint =
                pictures[1].GetComponentsInChildren<RefPoint2D>()[0];
            SetProperty(uncertainPoint, "confidence", 0.25f);
            SetProperty(pictures[0], "maximumNormalizedReprojectionError", 4f);

            bool success = ReconstructionInputCollector.TryCollect(out ReconstructionInput input, out string error);

            Assert.That(success, Is.True, error);
            Assert.That(input.Cameras[0].Confidence, Is.EqualTo(0.35).Within(0.0001));
            Assert.That(input.Cameras[1].Confidence, Is.EqualTo(0.6).Within(0.0001));
            Assert.That(input.Cameras[2].Confidence, Is.EqualTo(0.9).Within(0.0001));
            Assert.That(
                input.ObservationConfidences[8],
                Is.EqualTo(0.25).Within(0.0001));
            Assert.That(
                input.ObservationConfidences[0],
                Is.EqualTo(1.0).Within(0.0001));
            Assert.That(
                input.Options.MaximumNormalizedReprojectionError,
                Is.EqualTo(4.0).Within(0.0001));
        }

        /// <summary> 验证参考线端点可不对应，并按图片顺序进入原生输入。 </summary>
        [Test]
        public void TryCollect_AcceptsArbitraryLineHandlesWithMatchingIds()
        {
            RefPicture[] pictures = CreateValidSetup(8);
            RefLine2D axisLine =
                AddLine(pictures[0], 7, new Vector2(100f, 200f), new Vector2(900f, 350f), 0.8f);
            AddLine(pictures[1], 7, new Vector2(400f, 150f), new Vector2(1500f, 900f), 0.6f);
            AddLine(pictures[2], 7, new Vector2(200f, 1300f), new Vector2(700f, 300f), 0.4f);
            var lineSerialized = new SerializedObject(axisLine);
            lineSerialized.FindProperty("useAsResultXAxis").boolValue = true;
            lineSerialized.ApplyModifiedPropertiesWithoutUndo();

            bool success = ReconstructionInputCollector.TryCollect(
                out ReconstructionInput input,
                out string error);

            Assert.That(success, Is.True, error);
            Assert.That(input.LineIds, Is.EqualTo(new[] { 7 }));
            Assert.That(input.LineObservations, Has.Length.EqualTo(3));
            Assert.That(input.LineObservations[0].StartX, Is.Not.EqualTo(
                input.LineObservations[1].StartX).Within(0.001));
            Assert.That(input.LineObservations[0].Confidence, Is.EqualTo(0.8).Within(0.0001));
            Assert.That(input.LineObservations[1].Confidence, Is.EqualTo(0.6).Within(0.0001));
            Assert.That(input.LineObservations[2].Confidence, Is.EqualTo(0.4).Within(0.0001));
            Assert.That(input.ResultXAxisLineId, Is.EqualTo(7));
        }

        /// <summary> 验证三张图片必须为同一组参考线提供相同 ID。 </summary>
        [Test]
        public void TryCollect_RejectsMismatchedLineSet()
        {
            RefPicture[] pictures = CreateValidSetup(8);
            AddLine(pictures[0], 3, new Vector2(100f, 200f), new Vector2(800f, 300f));
            AddLine(pictures[1], 3, new Vector2(200f, 150f), new Vector2(1200f, 700f));

            bool success = ReconstructionInputCollector.TryCollect(out _, out string error);

            Assert.That(success, Is.False);
            StringAssert.Contains("RefLine2D ID 集合", error);
        }

        /// <summary> 验证三维连线直接持有两个参考点，不依赖结果根节点。 </summary>
        [Test]
        public void RefPointConnection3D_HoldsItsOwnEndpoints()
        {
            GameObject pointAObject = new("PointA", typeof(RefPoint3D));
            GameObject pointBObject = new("PointB", typeof(RefPoint3D));
            RefPoint3D pointA = pointAObject.GetComponent<RefPoint3D>();
            RefPoint3D pointB = pointBObject.GetComponent<RefPoint3D>();
            pointA.Initialize(2, 0.1f);
            pointB.Initialize(5, 0.2f);
            GameObject connectionObject = new("Connection", typeof(RefPointConnection3D));
            RefPointConnection3D connection =
                connectionObject.GetComponent<RefPointConnection3D>();

            connection.Initialize(pointA, pointB);

            Assert.That(connection.PointA, Is.SameAs(pointA));
            Assert.That(connection.PointB, Is.SameAs(pointB));
            Assert.That(connection.PointA.Id, Is.EqualTo(2));
            Assert.That(connection.PointB.Id, Is.EqualTo(5));
        }

        /// <summary> 验证径向增长的残差会生成超广角和畸变嫌疑提示。 </summary>
        [Test]
        public void Diagnostics_FlagsUltraWideRadialResidualPattern()
        {
            CreateValidSetup(8);
            Assert.That(
                ReconstructionInputCollector.TryCollect(out ReconstructionInput input, out string error),
                Is.True,
                error);

            const double focal = 500.0;
            const double depth = 2.0;
            ReconstructionNativeApi.CameraInput cameraInput = input.Cameras[0];
            double halfDiagonal = 0.5 * System.Math.Sqrt(
                cameraInput.Width * (double)cameraInput.Width +
                cameraInput.Height * (double)cameraInput.Height);
            double pixelScale = 1000.0 / System.Math.Max(cameraInput.Width, cameraInput.Height);
            var outputs = new ReconstructionNativeApi.CameraOutput[3];
            outputs[0] = new ReconstructionNativeApi.CameraOutput
            {
                FocalLengthPixels = focal,
                HorizontalFov = 2.0 * System.Math.Atan(cameraInput.Width / (2.0 * focal)) * Mathf.Rad2Deg,
                VerticalFov = 2.0 * System.Math.Atan(cameraInput.Height / (2.0 * focal)) * Mathf.Rad2Deg,
                RotationW = 1.0
            };

            var points = new ReconstructionNativeApi.PointOutput[input.PointIds.Length];
            for (int pointIndex = 0; pointIndex < points.Length; pointIndex++)
            {
                double radius = 0.1 + pointIndex * 0.09;
                double direction = pointIndex % 2 == 0 ? 1.0 : -1.0;
                double projectedX = cameraInput.Width * 0.5 + direction * radius * halfDiagonal;
                double radialResidual = 8.0 * radius * radius * radius;
                input.Observations[pointIndex] = new ReconstructionNativeApi.Observation
                {
                    X = projectedX + direction * radialResidual / pixelScale,
                    Y = cameraInput.Height * 0.5
                };
                points[pointIndex] = new ReconstructionNativeApi.PointOutput
                {
                    Id = input.PointIds[pointIndex],
                    PositionX = (projectedX - cameraInput.Width * 0.5) / focal * depth,
                    PositionZ = depth
                };
            }

            string report = ReconstructionCameraDiagnostics.Build(
                input,
                outputs,
                points,
                out bool hasWarning);

            Assert.That(hasWarning, Is.True);
            StringAssert.Contains("超广角", report);
            StringAssert.Contains("疑似未校正的镜头或全景畸变", report);
            StringAssert.Contains("畸变证据=高", report);
            StringAssert.Contains("最外圈参考点的系统性径向偏移约 3.7px", report);
            StringAssert.Contains("图片长边 0.31%", report);
            StringAssert.Contains("置信度≈0.25", report);
        }

        /// <summary> 验证高误差图片会用直白文字指出最应检查的点。 </summary>
        [Test]
        public void Diagnostics_ExplainsLocalizedPointErrorInPlainLanguage()
        {
            CreateValidSetup(8);
            Assert.That(
                ReconstructionInputCollector.TryCollect(out ReconstructionInput input, out string error),
                Is.True,
                error);

            const double focal = 900.0;
            const double depth = 2.0;
            ReconstructionNativeApi.CameraInput cameraInput = input.Cameras[0];
            var outputs = new ReconstructionNativeApi.CameraOutput[3];
            outputs[0] = new ReconstructionNativeApi.CameraOutput
            {
                FocalLengthPixels = focal,
                HorizontalFov =
                    2.0 * System.Math.Atan(cameraInput.Width / (2.0 * focal)) * Mathf.Rad2Deg,
                VerticalFov =
                    2.0 * System.Math.Atan(cameraInput.Height / (2.0 * focal)) * Mathf.Rad2Deg,
                RotationW = 1.0
            };

            var points = new ReconstructionNativeApi.PointOutput[input.PointIds.Length];
            for (int pointIndex = 0; pointIndex < points.Length; pointIndex++)
            {
                ReconstructionNativeApi.Observation observation = input.Observations[pointIndex];
                points[pointIndex] = new ReconstructionNativeApi.PointOutput
                {
                    Id = input.PointIds[pointIndex],
                    PositionX =
                        (observation.X - cameraInput.Width * 0.5) / focal * depth,
                    PositionY =
                        (cameraInput.Height * 0.5 - observation.Y) / focal * depth,
                    PositionZ = depth
                };
            }

            const int problemPointIndex = 5;
            ReconstructionNativeApi.Observation problem =
                input.Observations[problemPointIndex];
            double offsetX = problem.X - cameraInput.Width * 0.5;
            double offsetY = problem.Y - cameraInput.Height * 0.5;
            double radius = System.Math.Sqrt(offsetX * offsetX + offsetY * offsetY);
            input.Observations[problemPointIndex] = new ReconstructionNativeApi.Observation
            {
                X = problem.X - offsetY / radius * 60.0,
                Y = problem.Y + offsetX / radius * 60.0
            };

            string report = ReconstructionCameraDiagnostics.Build(
                input,
                outputs,
                points,
                out bool hasWarning);

            Assert.That(hasWarning, Is.True);
            StringAssert.Contains("主要问题是 Camera 0", report);
            StringAssert.Contains("主要由少数几个点拖累", report);
            StringAssert.Contains($"ID {input.PointIds[problemPointIndex]}≈60.0px", report);
            StringAssert.Contains("不像典型的镜头径向畸变", report);
        }

        [Test]
        public void TryCollect_RejectsDuplicateCameraId()
        {
            RefPicture[] pictures = CreateValidSetup(8);
            SetProperty(pictures[2], "cameraId", 1);

            bool success = ReconstructionInputCollector.TryCollect(out _, out string error);

            Assert.That(success, Is.False);
            StringAssert.Contains("唯一", error);
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
            StringAssert.StartsWith("ReconstructionNative/1.8.0", ReconstructionNativeApi.Version);
        }

        [Test]
        public void Solve_CreatesScaledHierarchyAndSecondSolveIsUndoable()
        {
            (RefPicture[] pictures, Vector3[] points, float knownDistance) = CreateSyntheticSetup();

            ReconstructionSceneSolver.Solve();

            GameObject firstResult = FindResultRoot();
            Assert.That(firstResult, Is.Not.Null);
            Transform cameraRoot = firstResult.transform.Find("Cameras");
            Transform pointRoot = firstResult.transform.Find("Points");
            Transform lineRoot = firstResult.transform.Find("Lines");
            Transform connectionRoot = firstResult.transform.Find("Connections");
            Assert.That(cameraRoot, Is.Not.Null);
            Assert.That(pointRoot, Is.Not.Null);
            Assert.That(lineRoot, Is.Not.Null);
            Assert.That(connectionRoot, Is.Not.Null);
            Assert.That(cameraRoot.childCount, Is.EqualTo(4));
            Assert.That(pointRoot.childCount, Is.EqualTo(points.Length));
            Assert.That(lineRoot.childCount, Is.EqualTo(1));
            Assert.That(connectionRoot.childCount, Is.EqualTo(1));
            RefLine3D solvedLine = lineRoot.GetChild(0).GetComponent<RefLine3D>();
            Assert.That(solvedLine.Id, Is.EqualTo(20));
            Assert.That(cameraRoot.GetChild(0).position.sqrMagnitude, Is.LessThan(0.000001f));
            Assert.That(
                Quaternion.Angle(cameraRoot.GetChild(0).rotation, Quaternion.identity),
                Is.LessThan(0.01f));

            RefPoint3D pointA = FindPoint(pointRoot, 100);
            RefPoint3D pointB = FindPoint(pointRoot, 129);
            Assert.That(
                Vector3.Distance(pointA.transform.position, pointB.transform.position),
                Is.EqualTo(knownDistance).Within(0.001f));
            RefPointConnection3D solvedConnection =
                connectionRoot.GetChild(0).GetComponent<RefPointConnection3D>();
            Assert.That(solvedConnection.PointA.Id, Is.EqualTo(100));
            Assert.That(solvedConnection.PointB.Id, Is.EqualTo(129));

            Vector3 lineDirectionBefore = solvedLine.Direction;
            Vector3 pointPositionBefore = pointA.transform.position;
            Quaternion rootRotation = Quaternion.Euler(17f, 39f, -11f);
            firstResult.transform.rotation = rootRotation;
            Assert.That(
                Vector3.Angle(solvedLine.Direction, rootRotation * lineDirectionBefore),
                Is.LessThan(0.001f));
            Assert.That(
                Vector3.Distance(pointA.transform.position, rootRotation * pointPositionBefore),
                Is.LessThan(0.0001f));
            int firstResultInstanceId = firstResult.GetInstanceID();

            ReconstructionSceneSolver.Solve();
            GameObject secondResult = FindResultRoot();
            Assert.That(secondResult, Is.Not.Null);
            Assert.That(secondResult, Is.Not.SameAs(firstResult));
            Assert.That(CountResultRoots(), Is.EqualTo(1));

            Undo.PerformUndo();
            GameObject restoredResult = FindResultRoot();
            Assert.That(restoredResult.GetInstanceID(), Is.EqualTo(firstResultInstanceId));
            Assert.That(CountResultRoots(), Is.EqualTo(1));
            Assert.That(pictures[0], Is.Not.Null);
        }

        /// <summary> 验证指定参考线会把结果整体对齐到世界 X 轴。 </summary>
        [Test]
        public void Solve_AlignsSelectedReferenceLineToWorldXAxis()
        {
            (RefPicture[] pictures, _, _) = CreateSyntheticSetup();
            RefLine2D axisLine = pictures[0].GetComponentInChildren<RefLine2D>();
            var lineSerialized = new SerializedObject(axisLine);
            lineSerialized.FindProperty("useAsResultXAxis").boolValue = true;
            lineSerialized.ApplyModifiedPropertiesWithoutUndo();

            ReconstructionSceneSolver.Solve();

            GameObject result = FindResultRoot();
            Assert.That(result, Is.Not.Null);
            RefLine3D solvedLine =
                result.transform.Find("Lines/RefLine_20").GetComponent<RefLine3D>();
            Assert.That(
                Mathf.Abs(Vector3.Dot(solvedLine.Direction.normalized, Vector3.right)),
                Is.GreaterThan(0.99999f));
            Assert.That(
                Quaternion.Angle(
                    result.transform.Find("Cameras/CameraPoint_0").localRotation,
                    Quaternion.identity),
                Is.LessThan(0.01f));
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

        /// <summary> 创建一个不含参考点的附加图片机位。 </summary>
        private static RefPicture CreateAdditionalPicture(int cameraId)
        {
            GameObject pictureObject = new(
                $"AdditionalPicture_{cameraId}",
                typeof(RectTransform),
                typeof(Canvas),
                typeof(RefPicture));
            RefPicture picture = pictureObject.GetComponent<RefPicture>();
            picture.RectTransform.sizeDelta = new Vector2(1600f, 900f);
            picture.RectTransform.position = new Vector3(cameraId * 2500f, -900f, 0f);
            SetProperty(picture, "cameraId", cameraId);
            return picture;
        }

        private static (RefPicture[] pictures, Vector3[] points, float knownDistance) CreateSyntheticSetup()
        {
            Vector2Int[] sizes =
            {
                new(4000, 3000),
                new(1920, 1080),
                new(3024, 4032),
                new(2560, 1440)
            };
            float[] fovs = { 52f, 68f, 44f, 58f };
            Vector3[] centers =
            {
                Vector3.zero,
                new(1.15f, 0.08f, 0.18f),
                new(-0.8f, -0.12f, 0.35f),
                new(0.45f, 0.25f, -0.25f)
            };
            var points = new Vector3[30];
            for (int index = 0; index < points.Length; index++)
            {
                points[index] = new Vector3(
                    Mathf.Sin(index * 1.71f) * 0.78f,
                    Mathf.Cos(index * 1.13f) * 0.52f,
                    4.1f + (index * 17 % 29) / 29f * 2.8f);
            }
            Vector3 linePoint = new(-0.2f, 0.1f, 5.2f);
            Vector3 lineDirection = new Vector3(0.9f, 0.25f, 0.15f).normalized;

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
                        3 => RotateY(RotateX(cameraPoint, -0.03f), 0.11f),
                        _ => cameraPoint
                    };
                    Vector2 pixel = new(
                        focal * cameraPoint.x / cameraPoint.z + sizes[cameraIndex].x * 0.5f,
                        focal * cameraPoint.y / cameraPoint.z + sizes[cameraIndex].y * 0.5f);
                    if (cameraIndex < 3 || pointIndex % 2 == 0)
                    {
                        AddPoint(picture, 100 + pointIndex, pixel);
                    }
                }

                if (cameraIndex < 3)
                {
                    float startOffset = -1.2f - cameraIndex * 0.25f;
                    float endOffset = 1.1f + cameraIndex * 0.35f;
                    Vector2 lineStart = ProjectSyntheticPoint(
                        linePoint + lineDirection * startOffset,
                        sizes[cameraIndex],
                        focal,
                        centers[cameraIndex],
                        cameraIndex);
                    Vector2 lineEnd = ProjectSyntheticPoint(
                        linePoint + lineDirection * endOffset,
                        sizes[cameraIndex],
                        focal,
                        centers[cameraIndex],
                        cameraIndex);
                    AddLine(picture, 20, lineStart, lineEnd);
                }
            }

            float knownDistance = Vector3.Distance(points[0], points[^1]);
            SetProperty(pictures[0], "scalePointIdA", 100);
            SetProperty(pictures[0], "scalePointIdB", 129);
            SetProperty(pictures[0], "scaleReferenceDistance", knownDistance);
            var pictureSerialized = new SerializedObject(pictures[0]);
            SerializedProperty connections = pictureSerialized.FindProperty("pointConnections");
            connections.arraySize = 1;
            connections.GetArrayElementAtIndex(0)
                .FindPropertyRelative("pointIdA").intValue = 100;
            connections.GetArrayElementAtIndex(0)
                .FindPropertyRelative("pointIdB").intValue = 129;
            pictureSerialized.ApplyModifiedPropertiesWithoutUndo();
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

        /// <summary> 使用合成相机把三维点投影到图片左上角坐标。 </summary>
        private static Vector2 ProjectSyntheticPoint(
            Vector3 point,
            Vector2Int size,
            float focal,
            Vector3 center,
            int cameraIndex)
        {
            Vector3 cameraPoint = point - center;
            cameraPoint = cameraIndex switch
            {
                1 => RotateY(cameraPoint, 0.18f),
                2 => RotateY(RotateX(cameraPoint, 0.04f), -0.15f),
                3 => RotateY(RotateX(cameraPoint, -0.03f), 0.11f),
                _ => cameraPoint
            };
            return new Vector2(
                focal * cameraPoint.x / cameraPoint.z + size.x * 0.5f,
                focal * cameraPoint.y / cameraPoint.z + size.y * 0.5f);
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

        /// <summary> 查找工具生成的普通结果根节点。 </summary>
        private static GameObject FindResultRoot()
        {
            foreach (GameObject root in EditorSceneManager.GetActiveScene().GetRootGameObjects())
            {
                if (root.name == "ReconstructionResult" &&
                    root.transform.Find("Cameras") != null)
                {
                    return root;
                }
            }
            return null;
        }

        /// <summary> 统计工具生成的普通结果根节点。 </summary>
        private static int CountResultRoots()
        {
            int count = 0;
            foreach (GameObject root in EditorSceneManager.GetActiveScene().GetRootGameObjects())
            {
                if (root.name == "ReconstructionResult" &&
                    root.transform.Find("Cameras") != null)
                {
                    count++;
                }
            }
            return count;
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

        /// <summary> 按图片像素坐标创建一条二维参考线操作段。 </summary>
        private static RefLine2D AddLine(
            RefPicture picture,
            int id,
            Vector2 startPixel,
            Vector2 endPixel,
            float confidence = 1f)
        {
            GameObject lineObject = new($"Line_{id}", typeof(RectTransform), typeof(RefLine2D));
            RectTransform line = lineObject.GetComponent<RectTransform>();
            line.SetParent(picture.transform, false);
            Rect rect = picture.RectTransform.rect;
            Vector2 localStart = new(rect.xMin + startPixel.x, rect.yMax - startPixel.y);
            Vector2 localEnd = new(rect.xMin + endPixel.x, rect.yMax - endPixel.y);
            Vector2 direction = localEnd - localStart;
            line.localPosition = (localStart + localEnd) * 0.5f;
            line.localRotation = Quaternion.Euler(
                0f,
                0f,
                Mathf.Atan2(direction.y, direction.x) * Mathf.Rad2Deg);
            line.sizeDelta = new Vector2(direction.magnitude, 8f);
            RefLine2D component = lineObject.GetComponent<RefLine2D>();
            SetProperty(component, "id", id);
            SetProperty(component, "confidence", confidence);
            return component;
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

        /// <summary> 写入测试对象的布尔序列化字段。 </summary>
        private static void SetProperty(Object target, string name, bool value)
        {
            var serializedObject = new SerializedObject(target);
            serializedObject.FindProperty(name).boolValue = value;
            serializedObject.ApplyModifiedPropertiesWithoutUndo();
        }
    }
}
