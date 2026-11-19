# FAST-Calib-ROS2-modified
# FAST-Calib ROS2 标定程序操作说明

基于 [FAST-Calib](https://github.com/hku-mars/FAST-Calib) 的 LiDAR-相机外参标定工具，已移植到 ROS2（Ubuntu 22.04 / ROS2 Humble）。

相比原版，本版本增加了**交互式滤波**：可以在 RViz 里直接拖动一个包围盒来框选标定板范围，实时看到检测结果，**不用再反复改 yaml 里的 x/y/z 范围**。

---

## 1. 标定板要求

标定板包含 **4 个 ArUco 二维码**（相机检测用）和 **4 个圆孔**（LiDAR 检测用）。

> ⚠️ 重要：4 个圆**必须是物理挖空的孔，或凸起的圆柱 / 反光环**。如果只是像二维码一样平面印刷的圆圈，激光雷达点云里没有几何边缘，就永远检测不到 4 个圆。

对应几何参数在 `config/qr_params.yaml` 里配置（见第 3 节）。

---

## 2. 数据准备

需要两样东西：

1. **一段 ros2 bag**（`sensor_msgs/msg/PointCloud2` 类型），里面是静态采集的标定板点云；
2. **一张同时拍摄的标定板图像**（`image.png`）。

目录示例：

```text
calib_data/mid360s/
├── lidar_points_0.db3     # ros2 bag 文件
├── metadata.yaml
└── image.png
```

---

## 3. 参数配置 `config/qr_params.yaml`

```yaml
fast_calib:
  ros__parameters:
    # 相机内参
    fx: 908.4497
    fy: 909.2491
    cx: 658.7119
    cy: 380.6899
    k1: -0.0319
    k2: 1.6510
    p1: 0.0030
    p2: 0.0005

    # 标定板几何
    marker_size: 0.20
    delta_width_qr_center: 0.55
    delta_height_qr_center: 0.35
    delta_width_circles: 0.5
    delta_height_circles: 0.4
    circle_radius: 0.12        # 圆孔半径（米）

    # 初始滤波范围（启动后可在 RViz 里拖动盒子改，这里只是初始值）
    x_min: -4.5
    x_max: 4.5
    y_min: -2.5
    y_max: 2.0
    z_min: -5.5
    z_max: 0.5

    # 输入
    lidar_topic: "/livox/lidar"
    bag_path: "/home/xch/fastcalib_ws/src/FAST-Calib-ROS2-dev/calib_data/mid360s"
    image_path: "/home/xch/fastcalib_ws/src/FAST-Calib-ROS2-dev/calib_data/mid360s/image.png"

    # 输出
    output_path: "/home/xch/fastcalib_ws/src/FAST-Calib-ROS2-dev/calib_data/mid360s"
```

要点：

- `bag_path` 是 ros2 bag 的**文件夹**（不是 .db3 文件）；
- `output_path` 是结果输出目录；
- `lidar_topic` 要和 bag 里点云话题一致；
- 改完 yaml 后要**重新 `colcon build`**（或直接改 `install/` 下的同名文件），因为 launch 读取的是 install 空间里的配置。

---

## 4. 编译

```bash
cd ~/fastcalib_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fast_calib
source install/setup.bash
```

---

## 5. 启动

```bash
ros2 launch fast_calib calib.launch.py
```

会自动打开 RViz 并加载 `rviz_cfg/fast_livo2.rviz`（已配好各话题显示和交互标记）。

如果不想开 RViz：`ros2 launch fast_calib calib.launch.py rviz:=false`。

手动运行（不用 launch）时，**必须带节点重映射和参数文件**，否则会读到默认路径报图像加载失败：

```bash
ros2 run fast_calib fast_calib --ros-args -r __node:=fast_calib --params-file <你的 qr_params.yaml 路径>
```

---

## 6. RViz 交互操作（核心）

打开 RViz 后：

1. **`raw_cloud`（灰白点云）** 是完整的 bag 点云，用来找标定板在哪。确认左侧 Displays 里 `raw_cloud` 勾选、`Global Options → Fixed Frame` 是 `map`。

2. **绿色半透明盒子 = 滤波范围**，只对这个盒子内的点云做检测：
   - **拖动**盒子 → 移动位置；
   - **右键盒子** → 按轴调整大小：
     ```text
     调整 X 轴 / Y 轴 / Z 轴
       ├── +0.1 m / -0.1 m
       ├── +0.5 m / -0.5 m
       ├── +1 m   / -1 m
       └── +5 m   / -5 m
     ```

3. **每次调整都会自动重跑**：滤波 → 平面分割 → 圆孔检测 → 外参求解。RViz 里的 `filtered_cloud`、`plane_cloud`、`center_cloud` 会实时刷新，终端同时打印结果。

4. **目标**：让绿色盒子刚好把整块标定板（含 4 个圆孔）框住，终端显示：

   ```text
   Number of edge clusters: 4
   [Result] RMSE: 0.0xxx m
   [Result] Extrinsic parameters T_cam_lidar:
   ...
   [Result] Calibration results saved to .../calib_result.txt
   ```

   RMSE 越小越好（一般几厘米以内）。

---

## 7. 保存与输出

| 文件 | 说明 | 何时生成 |
|------|------|----------|
| `calib_result.txt` | **外参**（Rcl/Pcl）+ 相机内参，下游 FAST-LIVO2 用 | 每次得到有效标定自动更新 |
| `colored_cloud.pcd` / `.ply` | 用外参投影上色的彩色点云 | 初始运行 + 右键「保存标定结果」 |
| `qr_detect.png` | 二维码检测可视化 | 每次保存外参时 |
| `filtered/plane/aligned/edge_cloud.ply` | 中间点云，排查用 | 右键「保存调试点云」或初始运行 |

右键菜单里另外两个功能：

- **保存标定结果 (Save result)**：把当前结果连同彩色点云一起写盘；
- **保存调试点云 (Debug save)**：按当前包围盒重跑，并保存 `filtered/plane/aligned/edge` 中间点云（排查检测问题用）。

所有文件都写到 yaml 里的 `output_path` 目录。

---

## 8. 常见问题

**Q1：RViz 里看不到点云？**
- 确认 `raw_cloud` 显示项 Enabled、Fixed Frame = `map`；
- 终端里 `ros2 topic hz /raw_cloud` 看是否在收；如果没，重新启动节点（`raw_cloud` 会持续发布）。

**Q2：检测不到 4 个圆？**
- 确认标定板上的 4 个圆是**物理挖空/凸起**的（见第 1 节）；
- 调整绿色盒子，让整块板子（含 4 个圆孔）都在盒内，且不要把别的平面（墙/地）框进来；
- 看终端 `circle fit: ... fitted radius ...`：如果拟合半径和 `circle_radius` 差很多，把 yaml 里的 `circle_radius` 改成实际半径。

**Q3：拖动时报 `Plane segmentation failed ... no plane in box`？**
- 说明盒子当前框到的区域没有足够大的平面（框空了）。把盒子拖回标定板上即可，**不会崩**。

**Q4：点云看起来上下颠倒（地板在上、天花板在下）？**
- 正常现象，是雷达坐标系 Z 轴朝向导致的，**不影响标定**。想转正的话可以在 RViz 里旋转视角。

**Q5：手动 `ros2 run` 报 `Loading the image ... failed`？**
- 忘了 `-r __node:=fast_calib` 和 `--params-file`，导致读到了默认参数路径。

**Q6：改了 `config/qr_params.yaml` 不生效？**
- launch 读的是 install 空间的配置，改完源码里的 yaml 后要重新 `colcon build`（或直接改 `install/fast_calib/share/fast_calib/config/qr_params.yaml`）。

---

## 附：算法流程（简述）

1. 直通滤波（绿色盒子）截取点云 → 体素降采样；
2. RANSAC 平面分割出标定板平面，旋转对齐到 Z=0；
3. 边界提取 + 欧式聚类，得到 4 个圆孔边缘；
4. 对每个圆孔拟合圆，得到 4 个圆心（LiDAR 侧）；
5. ArUco 检测得到 4 个圆心（相机侧）；
6. SVD 求解 LiDAR→相机外参，输出 Rcl / Pcl 和 RMSE。
