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

三个插帧入口会同时启动 `visualization_node` 和 `interpolation_display_node` 两个
ROS 2 进程。Open3D 源渲染窗口在生成进程中保持隐藏，并且只在新网格到达时渲染一次；
生成完成的中间帧按 deadline 发布到本机
`interpolated_frames/<camera_id>` 图像话题。发布端和显示端使用 reliable KeepAll
DDS；显示进程用有界 FIFO 保存已经进入回调的帧，队列满时通过 DDS 向发布端反压。
ROS 接收由独立线程负责，
显示进程主线程只负责 OpenCV 窗口的创建、`imshow()`、事件处理和销毁。因此 HighGUI
阻塞不会再阻塞 ROS 回调，也不会因单帧缓存覆盖而静默丢帧。

Worker 会先把完整 `FrameSequence` 放入 ready 队列，再通知输出调度线程。输出调度线程
按下一帧截止时间唤醒；10 ms 轮询只用于等待 DDS 发现尚未连接的显示订阅者。一次调度
最多发布一帧，即使已经迟到也不会跨过中间帧；下一帧从本次实际发布时间重新计时。因此
消费端过慢时表现为播放时间线变长和 reliable 反压，而不是从一段中间跳到下一段。

显示窗口创建后会恢复多线程改造前使用的 `cv::startWindowThread()` 事件驱动方式。后端
支持时不再调用可能长时间阻塞的 `waitKey(1)`；后端不支持时才使用 `waitKey()` 回退，
此时 ROS 接收线程和 FIFO 仍会继续工作，并通过日志报告队列积压与反压次数。

## 版本对照

| 入口 | 中间帧 | 光流分辨率 | 光流方向 | 用途 |
| --- | ---: | ---: | --- | --- |
| `v_vis.launch.py` | 0 | 无 | 无 | 原始可视化基线 |
| `v_dis_ultrafast.launch.py` | 4 | 1/4 | 单向 | 保留的低开销基础版 |
| `v_dis_bidirectional.launch.py` | 4 | 1/4 | 双向 + 一致性掩码 | 当前统一使用的 x5 版本 |
| `v_dis_quality_x20.launch.py` | 19 | 1/2 | 单向 | 保留的历史压力测试，不作为当前运行方案 |

不要同时启动多个可视化 launch；需要比较时，先停止当前可视化节点再切换。

## FPS 统计与日志

所有版本默认开启 FPS 日志，统计窗口为 5 秒：

```yaml
fps_logging_enabled: true
fps_logging_interval_sec: 5.0
```

原始可视化输出：

```text
[FPS] Display "..." (no interpolation): presented 0.50 FPS | 6.0 s window | max display gap 2010 ms | superseded meshes +0 (total 3).
```

这里统计的是 Open3D 完成新网格窗口更新的频率，不把没有新内容的 UI 空转计为新帧。

插帧版本同时输出：

```text
[GEN] Bidirectional DIS + consistency mask: 4 intermediate display frames in 190.5 ms
[SRC] Rendered "...": 1.80 FPS | 5.0 s window | max gap 610 ms | superseded meshes +0 (total 0).
[TX] Interpolated "...": published playback 9.0 FPS | published effective 8.8 FPS | 5.0 s window | max publish gap 150 ms | coalesced +0 (total 0) | queue 0 pending / 0 ready / 4 active | worker idle | lag 0.0 ms.
[FPS] Display "...": received 8.8 FPS (+44, total 88) | presented 8.8 FPS (+44, total 88) | 5.0 s window | max display gap 151 ms | HighGUI max imshow 1.2 ms / waitKey 0.0 ms | queue 0 current / 2 max / 10 capacity | backpressure waits +0 (total 0) | missing +0 (total 0) | out-of-order +0 (total 0) | unsequenced +0 (total 0) | rejected +0 (total 0).
```

- `[TX]` 只统计生成进程调用图像发布的频率，不代表窗口实际显示帧率；
- `[FPS] Display` 的 `received` 是显示节点回调实际收到的帧率；
- `presented` 是显示主线程实际提交给 HighGUI 的帧率，是正式显示端 FPS；
- `max display gap` 是相邻 presented 帧的最大时间间隔；
- `HighGUI max imshow / waitKey` 用于定位窗口后端是否阻塞显示消费；
- `missing` 根据发布端写入图像消息头的单调编号统计，表示帧在发布后、显示回调前丢失；
- `queue current / max / capacity` 是显示进程 FIFO 的当前、窗口内最大和容量；
- `backpressure waits` 表示 FIFO 已满、ROS 回调等待显示端腾出位置的次数；
- `out-of-order / unsequenced / rejected` 分别表示乱序、缺少有效诊断编号和图像校验失败；
- `+N (total M)`：`N` 是当前统计窗口新增次数，`M` 是进程启动后的累计次数；
- `pending / ready / active`：待生成的真实帧对、已生成序列，以及当前播放序列剩余帧数；
- `worker` 和 `lag`：生成线程是否忙碌，以及当前活动帧超过播放截止时间的毫秒数；
- `[GEN]`：每对真实帧生成的中间显示帧数量和总耗时。

待生成队列已满时，`coalesced` 表示新真实帧被合并进最新待处理帧对。合并后的播放
周期使用被合并源间隔的平均值，而不是把多个源间隔总和当成一个周期，因此连续输入
间隔相近时不会因为一次合并把名义播放帧率减半。

`lag` 是相对播放截止时间的调度迟到，不是“最新真实帧到达后经过了多久”。迟到不会
触发跳帧或重置；调度器提交当前帧后从实际提交时刻安排下一帧。

OpenCV HighGUI 没有跨平台的“显示器已经扫描并呈现此帧”回调，因此 `presented` 的
测量边界是应用把新帧提交给 HighGUI，不等同于显示器物理扫描时刻。正常运行时
`TX ≈ received ≈ presented`、`missing = 0`、`backpressure waits = 0`；若
`received ≈ TX` 但 `presented` 较低并且 FIFO 上升，瓶颈仍在 HighGUI。

## 自动化验证

插值器测试覆盖合并时间间隔、显示帧所有权、ready 入队通知、迟到时逐帧连续播放、
ready 队列反压、队列状态与播放延迟、消息时间戳回退、尺寸变化重置，以及双向一致性路径；另有
测试验证连续图像、带 stride 的 ROI 和畸形 `sensor_msgs/Image` 转换：

```bash
colcon test --packages-select visualization
colcon test-result --verbose
```
