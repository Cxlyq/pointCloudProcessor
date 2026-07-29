# 可视化插帧方案索引

## 约束

本目录中的方案必须遵守以下边界：

- 不修改点云输入、地面分割、聚类和重建流程；
- 不修改现有 ROS 消息和话题；
- `visualization_node` 只额外保存上一张真实渲染图；
- 中间帧只用于显示，不发布为新的点云或网格；
- 默认固定相机，保证相邻渲染图具有相同视角；
- 允许约一个真实帧周期的显示延迟。

当前采用固定倍帧逻辑，不设置绝对目标 FPS。基础版在每两个真实帧之间插入 4 张虚拟帧，
清晰度测试版插入 19 张。虚拟帧的播放间隔都根据两张真实网格的实际到达时间自动计算。

## 实施顺序

| 阶段 | 文档 | 状态 | 修改范围 | 计算开销 | 适用目的 |
| --- | --- | --- | --- | --- | --- |
| 1 | [单向低分辨率 DIS](01_dis_ultrafast_single_direction.md) | 已实现 | 仅可视化包 | 低 | 先验证两秒跨度的基础效果 |
| 1Q | [单向 DIS 清晰度 x20 测试版](01b_dis_quality_x20.md) | 已实现 | 仅新增配置和 launch | 中 | 提升帧数并改善光流精度 |
| 2 | [双向 DIS](02_dis_bidirectional.md) | 已实现 | 仅可视化包 | 中 | 降低轮廓重影 |
| 3 | [深度边缘辅助 DIS](03_dis_depth_edge_guided.md) | 待实现 | 仅可视化包 | 中 | 改善白模弱纹理问题 |
| 4 | [全局仿射插帧](04_global_affine.md) | 待实现 | 仅可视化包 | 最低 | 画面主要是统一运动时使用 |
| 5 | [全局预对齐加 DIS](05_global_alignment_plus_dis.md) | 待实现 | 仅可视化包 | 中 | 大位移时降低 DIS 搜索难度 |
| 延后 | [RIFE 神经插帧](90_rife_deferred.md) | 不在当前计划 | 仅可视化包，但引入模型运行时 | 高 | 弱硬件方案无法满足质量时再评估 |

## 已排除方案

下列方案会改变流水线或重建数据表达，不再实施：

- OBB 目标跟踪和三维网格插值；
- 固定拓扑地面插值；
- 新增 `SceneFrame` 或跟踪目标消息；
- 在原始点云层执行 ICP、场景流或最近邻插帧；
- 对拓扑不同的相邻网格直接做顶点插值。

## 原始显示与插帧显示

原始显示方式保持不变：

```bash
ros2 launch visualization v_vis.launch.py
```

第一阶段插帧显示：

```bash
ros2 launch visualization v_dis_ultrafast.launch.py
```

第二阶段双向 DIS 插帧显示：

```bash
ros2 launch visualization v_dis_bidirectional.launch.py
```

单向 DIS 清晰度 x20 测试版：

```bash
ros2 launch visualization v_dis_quality_x20.launch.py
```

三个插帧入口默认都只显示插值结果窗口；Open3D 源渲染窗口在后台隐藏，并且只在新
网格到达时渲染一次，避免占用插值播放线程。隐藏渲染器使用显式离屏渲染后再截图，
不会读取未渲染的黑色前缓冲。

## 版本对照

| 入口 | 中间帧 | 光流分辨率 | 光流方向 | 用途 |
| --- | ---: | ---: | --- | --- |
| `v_vis.launch.py` | 0 | 无 | 无 | 原始可视化基线 |
| `v_dis_ultrafast.launch.py` | 4 | 1/4 | 单向 | 保留的低开销基础版 |
| `v_dis_bidirectional.launch.py` | 4 | 1/4 | 双向 | 对照轮廓重影 |
| `v_dis_quality_x20.launch.py` | 19 | 1/2 | 单向 | 本轮帧数与清晰度测试 |

不要同时启动多个可视化 launch；需要比较时，先停止当前可视化节点再切换。

## FPS 统计与诊断日志

所有版本默认开启诊断日志，统计窗口为 5 秒：

```yaml
fps_logging_enabled: true
fps_logging_interval_sec: 5.0
```

日志按处理阶段拆分：

- `[RX]`：ROS 回调实际接收 Mesh 的帧率、回调构建 Mesh 的耗时和交接前被覆盖的帧数；
- `[DISPLAY-RAW]`：不插帧模式下，新内容到达 Open3D 显示边界的帧率及各阶段耗时；
- `[SOURCE]`：插帧模式下，成功完成 Open3D 渲染和截图的真实帧率；
- `[RENDER]`：等待、窗口事件、Geometry 更新、渲染截图、格式转换、校验和提交的平均/最大耗时；
- `[GEN]`：DIS 序列号、两端真实帧编号、端点跨度、合并数、中间帧数和计算耗时；
- `[DISPLAY]`：`imshow()` 提交帧率、批内播放帧率、最大间隔、调用耗时、调度逾期和顺序错误；
- `[PIPE]`：最近提交的序列/帧编号以及 pending、ready、active、worker 和累计合并状态；
- `[HIGHGUI]`：仅在 `startWindowThread()` 不可用时统计 `waitKey()` 事件泵耗时。

典型插帧日志如下：

```text
[RX] Accepted mesh: 1.96 FPS | 5.1 s window | max gap 530 ms | callback avg/max 18.0/24.0 ms
[RENDER] "...": 10 source frames | age avg/max 12.0/35.0 ms | geometry 20.0/31.0 | capture 85.0/122.0 ms
[GEN] Bidirectional DIS: sequence 12 source 12->13 | span 505.0 ms | coalesced 0 | 4 intermediate display frames in 75.0 ms
[DISPLAY] "...": imshow-submit 9.5 FPS | playback 9.7 FPS | max gap 135 ms | imshow avg/max 2.0/8.0 ms
[PIPE] Last S12 5/5 source 12->13 | queue 0 pending / 0 ready / 0 active | worker idle
```

`DISPLAY` 统计的是程序成功完成 `imshow()` 调用的提交边界。OpenCV HighGUI 没有跨平台的
“显示器已经扫描并呈现此帧”回调，因此它不能单独证明物理屏幕刷新了每一张图。短时间核对
物理刷新时，可以把对应插帧配置中的以下参数改成 `true`：

```yaml
interpolation_diagnostic_overlay: true
```

画面左上角将显示 `S<序列号> <当前帧>/<总帧数>`。角标只应在短时验证时启用，因为复制和
绘制角标本身会产生少量额外开销；正式性能测试保持 `false`。
