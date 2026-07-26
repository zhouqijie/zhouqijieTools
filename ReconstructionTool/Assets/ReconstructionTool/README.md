# Unity 多视图稀疏重建工具

该工具使用 PoseLib 2.0.5、Ceres Solver 2.2.0 和 Eigen 3.4.0，从至少三张不同机位照片及人工对应点、可选参考线，恢复全部相机、独立焦距/FOV、稀疏三维点和三维直线。当前仅支持 Windows x64 UnityEditor，不生成网格。

## 场景设置

1. 创建至少三个 Canvas，使每个 `RectTransform` 的宽高比与完整原图一致；建议直接使用原图像素宽高。
2. 在 Canvas 同一对象添加 `RefPicture`，将 `Camera ID` 从 0 开始连续编号，并配置各自的垂直 FOV 搜索范围和图片置信度。
3. Camera 0～2 标记相同的至少 8 个基础点；每个 Camera 3+ 标记至少 4 个可见基础点，建议 6 个以上。
4. 不希望某个 Camera 3+ 改变公共三维结构时，可勾选其 `仅求相机位姿`。
5. 可在 Camera 0～2 中创建完整的 `RefLine2D` 集合；Camera 3+ 只需标出其中实际可见的参考线。
6. 如需定义结果的水平方向，可在一条 `RefLine2D` 上勾选“作为结果 X 轴”。
7. Camera 0 的 `RefPicture` 中指定两个尺度点 ID 和它们的真实距离。
8. 在任意一个 `RefPicture` Inspector 中点击 `Solve`。

成功后会创建：

```text
ReconstructionResult
├── Cameras
│   ├── CameraPoint_0
│   ├── CameraPoint_1
│   ├── CameraPoint_2
│   └── ...
├── Points
│   └── RefPoint_{ID}
├── Connections
│   └── Connection_{ID}_{ID}
└── Lines
    └── RefLine_{ID}
```

`ReconstructionResult` 是不挂载脚本的普通层级容器。相机、点、点连线和参考线均由各自组件与 Transform 自行表达。再次 Solve 只会在新解成功后替换旧结果，整个替换可用一次 Unity Undo 撤销。

## 数据要求

- Camera 0～2 必须来自同一静止场景，并具有至少 8 个完全一致且不重复的点 ID；建议 15 个以上。
- Camera 3+ 可以拍摄不完整，每个机位仍需提供至少 4 个可见基础点。两个以上参与公共重建的附加机位可以使用相同的新 ID 标记 Camera 0～2 完全看不到的新点，求解器会在附加相机初始化后恢复其三维位置。
- `仅求相机位姿` 只允许 Camera 3+ 使用：公共三维点和直线求解完成后保持固定，该机位的点线只优化自己的位置、旋转和 FOV。
- 附加新点不能计入相机初始化所需的 4 个基础点，并且至少要被两个参与公共重建的附加机位观测；只有一个机位看到的新点无法确定深度。
- 仅求位姿的机位不能参与生成附加新点；新 ID 必须先由至少两个参与公共重建的附加机位生成。
- 参考线是可选的辅助约束，不能代替初始化所需的至少 8 个参考点；同一 Line ID 必须表示同一条物理直线。
- 附加相机仍先用至少 4 个基础点执行 P4Pf 初始化，再使用 Camera 0～2 生成的三维参考线细化本机位；参考线会参与位置、旋转和 FOV 优化，但不能替代 4 个基础点。
- 每个 `RefPoint2D` 和 `RefLine2D` 都可以设置独立置信度；实际求解权重为“所属图片置信度 × 该点或线的置信度”。
- 最多只能指定一个 Line ID 作为结果 X 轴；求解后对应 `RefLine3D` 会与世界 `+X/-X` 平行，但不会被平移到世界原点。
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
