# Unity 三视图稀疏重建工具

该工具使用 PoseLib 2.0.5、Ceres Solver 2.2.0 和 Eigen 3.4.0，从三张不同相机拍摄的照片及人工对应点恢复三台相机、独立焦距/FOV 和稀疏三维点。第一版仅支持 Windows x64 UnityEditor，不生成网格。

## 场景设置

1. 创建恰好三个 Canvas，并把每个 Canvas 的 `RectTransform` 宽高设成对应原图的像素宽高。
2. 在 Canvas 同一对象添加 `RefPicture`，将 `Camera ID` 分别设成 0、1、2，并配置各自的垂直 FOV 搜索范围。
3. 在每个 Canvas 下创建相同数量的 UI 子对象并添加 `RefPoint2D`。同一个物理点在三张图中使用相同 ID。
4. Camera 0 的 `RefPicture` 中指定两个尺度点 ID 和它们的真实距离。
5. 在任意一个 `RefPicture` Inspector 中点击 `Solve`。

成功后会创建：

```text
ReconstructionResult
├── Cameras
│   ├── CameraPoint_0
│   ├── CameraPoint_1
│   └── CameraPoint_2
└── Points
    └── RefPoint_{ID}
```

再次 Solve 只会在新解成功后替换旧结果，整个替换可用一次 Unity Undo 撤销。

## 数据要求

- 三张图必须来自静止场景，至少 8 个完全一致且不重复的点 ID；建议 15 个以上。
- 点应有明显的非共面深度，相机之间应有足够平移视差，避免纯旋转和极小基线。
- 图片必须按原比例完整显示，不能非等比拉伸或做未知的非中心裁剪。
- 使用针孔模型，每张图独立 `fx=fy`、主点固定在中心，不估计镜头畸变；广角/鱼眼图片应先校正。

## 构建原生插件

安装 Visual Studio 的“使用 C++ 的桌面开发”工作负载，然后在 PowerShell 执行：

```powershell
.\Native\ReconstructionNative\Build-Native.ps1
```

脚本按固定 commit 下载依赖，构建静态 CRT 的 Release DLL，运行原生合成测试，再把 DLL/PDB 放入 `Assets/Plugins/x86_64`。如依赖已离线下载，可传入 `-DependencyRoot <目录>`，其中包含 `PoseLib`、`ceres-solver` 和 `eigen` 三个子目录。

## 坐标约定

输入点使用每个图片 `RectTransform.rect` 的左上角为像素原点。转换使用 `InverseTransformPoint`，因此 Canvas 的 pivot、整体缩放和旋转不会破坏坐标。原生层把计算机视觉坐标转换为 Unity 左手坐标；Camera 0 固定在世界原点且旋转为单位旋转。
