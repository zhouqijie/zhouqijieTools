using System;
using System.Runtime.InteropServices;
using System.Text;

namespace ReconstructionTool.Editor
{
    internal static class ReconstructionNativeApi
    {
        private const string LibraryName = "ReconstructionNative";

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
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct Observation
        {
            public double X;
            public double Y;
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
        internal struct SolveReport
        {
            public int Status;
            public int PointCount;
            public int InlierCount;
            public double NormalizedReprojectionRms;
            public double MedianTriangulationAngle;
            public double AppliedScale;
        }

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr RT_GetVersion();

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        private static extern int RT_SolveThreeView(
            [In] CameraInput[] cameras,
            [In] int[] pointIds,
            [In] Observation[] observations,
            int pointCount,
            ref SolveOptions options,
            [In, Out] CameraOutput[] cameraOutputs,
            [In, Out] PointOutput[] pointOutputs,
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
            Observation[] observations,
            SolveOptions options,
            out CameraOutput[] cameraOutputs,
            out PointOutput[] pointOutputs,
            out SolveReport report,
            out string error)
        {
            cameraOutputs = new CameraOutput[3];
            pointOutputs = new PointOutput[pointIds.Length];
            StringBuilder buffer = new(4096);
            int status = RT_SolveThreeView(
                cameras,
                pointIds,
                observations,
                pointIds.Length,
                ref options,
                cameraOutputs,
                pointOutputs,
                out report,
                buffer,
                buffer.Capacity);
            error = buffer.ToString();
            return (Status)status;
        }
    }
}
