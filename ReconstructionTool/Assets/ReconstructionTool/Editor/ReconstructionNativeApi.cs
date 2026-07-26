using System;
using System.Runtime.InteropServices;
using System.Text;

namespace ReconstructionTool.Editor
{
    internal static class ReconstructionNativeApi
    {
        private const string LibraryName = "ReconstructionNative_1_8";

        internal enum Status
        {
            Success = 0,
            InvalidArgument = 1,
            DegenerateGeometry = 2,
            InitializationFailed = 3,
            OptimizationFailed = 4,
            AmbiguousSolution = 5,
            HighReprojectionError = 6,
            InternalError = 7
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct CameraInput
        {
            public int Width;
            public int Height;
            public double MinimumVerticalFov;
            public double MaximumVerticalFov;
            public double Confidence;
            public int PoseOnly;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct Observation
        {
            public double X;
            public double Y;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct LineObservation
        {
            public double StartX;
            public double StartY;
            public double EndX;
            public double EndY;
            public double Confidence;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct SolveOptions
        {
            public int ScalePointIdA;
            public int ScalePointIdB;
            public double KnownScaleDistance;
            public double MaximumNormalizedReprojectionError;
            public int RandomSeed;
            public int MaximumCandidates;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct CameraOutput
        {
            public double FocalLengthPixels;
            public double HorizontalFov;
            public double VerticalFov;
            public double PositionX;
            public double PositionY;
            public double PositionZ;
            public double RotationX;
            public double RotationY;
            public double RotationZ;
            public double RotationW;
            public double ReprojectionRmsPixels;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct PointOutput
        {
            public int Id;
            public double PositionX;
            public double PositionY;
            public double PositionZ;
            public double ReprojectionRmsPixels;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct LineOutput
        {
            public int Id;
            public double PointX;
            public double PointY;
            public double PointZ;
            public double DirectionX;
            public double DirectionY;
            public double DirectionZ;
            public double ReprojectionRmsPixels;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct SolveReport
        {
            public int Status;
            public int PointCount;
            public int InlierCount;
            public int LineCount;
            public double NormalizedReprojectionRms;
            public double NormalizedLineRms;
            public double MedianTriangulationAngle;
            public double AppliedScale;
        }

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr RT_GetVersion();

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        private static extern int RT_SolveMultiView(
            [In] CameraInput[] cameras,
            int cameraCount,
            [In] int[] pointIds,
            [In] Observation[] observations,
            [In] byte[] observationVisibility,
            [In] double[] observationConfidences,
            int pointCount,
            int basePointCount,
            [In] int[] lineIds,
            [In] LineObservation[] lineObservations,
            [In] byte[] lineObservationVisibility,
            int lineCount,
            ref SolveOptions options,
            [In, Out] CameraOutput[] cameraOutputs,
            [In, Out] PointOutput[] pointOutputs,
            [In, Out] LineOutput[] lineOutputs,
            out SolveReport report,
            StringBuilder errorBuffer,
            int errorBufferCapacity);

        internal static string Version
        {
            get
            {
                IntPtr pointer = RT_GetVersion();
                return Marshal.PtrToStringAnsi(pointer) ?? "unknown";
            }
        }

        internal static Status Solve(
            CameraInput[] cameras,
            int[] pointIds,
            int basePointCount,
            Observation[] observations,
            byte[] observationVisibility,
            double[] observationConfidences,
            int[] lineIds,
            LineObservation[] lineObservations,
            byte[] lineObservationVisibility,
            SolveOptions options,
            out CameraOutput[] cameraOutputs,
            out PointOutput[] pointOutputs,
            out LineOutput[] lineOutputs,
            out SolveReport report,
            out string error)
        {
            cameraOutputs = new CameraOutput[cameras.Length];
            pointOutputs = new PointOutput[pointIds.Length];
            lineOutputs = new LineOutput[lineIds.Length];
            StringBuilder buffer = new(32768);
            int status = RT_SolveMultiView(
                cameras,
                cameras.Length,
                pointIds,
                observations,
                observationVisibility,
                observationConfidences,
                pointIds.Length,
                basePointCount,
                lineIds,
                lineObservations,
                lineObservationVisibility,
                lineIds.Length,
                ref options,
                cameraOutputs,
                pointOutputs,
                lineOutputs,
                out report,
                buffer,
                buffer.Capacity);
            error = buffer.ToString();
            return (Status)status;
        }
    }
}
