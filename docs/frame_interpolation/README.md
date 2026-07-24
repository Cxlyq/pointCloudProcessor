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

## FPS 统计与日志

所有版本默认开启 FPS 日志，统计窗口为 5 秒：

```yaml
fps_logging_enabled: true
fps_logging_interval_sec: 5.0
```

原始可视化输出：

```text
[FPS] Source "...": 0.50 FPS | 6.0 s window | max gap 2010 ms.
```

这里统计的是完成新网格渲染的频率，不把没有新内容的 UI 空转计为新帧。

插帧版本同时输出：

```text
[GEN] Single-direction DIS: 19 intermediate display frames in 320.5 ms
[FPS] Source "...": 0.50 FPS | 6.0 s window | max gap 2010 ms.
[FPS] Interpolated "...": playback 9.8 FPS | effective 8.7 FPS | 5.1 s window | max gap 410 ms.
```

- `playback FPS`：同一批中间帧连续播放时，根据实际提交间隔计算，更接近运动时的体感；
- `effective FPS`：把批次间等待、计算和排队的停顿也计入，通常更低；
- `max gap`：统计窗口内相邻输出帧的最大间隔，用于识别平均 FPS 掩盖的卡顿；
- `[GEN]`：每对真实帧生成的中间显示帧数量和总耗时。

OpenCV HighGUI 没有跨平台的“显示器已经扫描并呈现此帧”回调，因此插值 FPS 的测量点是
`imshow()` 完成提交的时刻。相比旧统计，新统计不再因两秒空档重置，并把批内播放速度
与包含停顿的长期有效速度分开报告。源网格渲染耗时和 HighGUI 后端信息已降为 DEBUG，
默认 INFO 日志主要保留 `[GEN]`、`[FPS]`、警告和错误。
