# 阶段 1Q：单向 DIS 清晰度测试版

状态：已实现，可与阶段 1 基础版直接对照。

## 目的

这个版本不替换原有的 `v_dis_ultrafast.launch.py`。它继续使用单向
`DISOpticalFlow::PRESET_ULTRAFAST`，只调整两个参数：

- 每两个真实帧之间由插入 4 张改为插入 19 张，长期帧数约为原始的 20 倍；
- 光流计算分辨率由原图的 1/4 提升到 1/2，改善轮廓位移估计。

若真实帧间隔约为 2 秒，即原始帧率约为 0.5 FPS，20 倍后理论显示帧率约为
10 FPS。这里仍然采用固定插帧数量，不设置目标 FPS；播放步长始终根据相邻真实帧
的实际到达间隔计算。

## 参数

配置文件：`src/visualization/config/v_dis_quality_x20_config.yaml`

```yaml
interpolation_intermediate_frames: 19
interpolation_flow_scale: 0.5
interpolation_dis_preset: "ultrafast"
interpolation_bidirectional_flow: false
interpolation_timing_source: "message_stamp"
interpolation_pending_pair_capacity: 3
interpolation_ready_sequence_capacity: 2
interpolation_highgui_event_mode: "wait_key"
```

19 张中间帧复用同一份光流，不会执行 19 次光流计算。相对基础版，新增开销主要来自：

- 1/2 分辨率的光流像素数约为 1/4 分辨率的 4 倍；
- 每对真实帧需要执行更多次全分辨率 Warp 和融合。

## 构建与运行

首次拉取新增 launch/config 后，在工作空间根目录执行：

```bash
colcon build --packages-up-to visualization
source install/setup.bash
ros2 launch visualization v_dis_quality_x20.launch.py
```

默认只显示：

`Real-time Render - Front View - DIS Quality x20`

不要与其他可视化 launch 同时运行。

## 对照版本

基础单向版仍保留，运行方式：

```bash
ros2 launch visualization v_dis_ultrafast.launch.py
```

它仍使用插入 4 张、1/4 分辨率、单向 ULTRAFAST，不受本版本影响。

## 试用时记录

- 日志中一对真实帧生成 19 张中间帧的耗时；
- `[SRC-TX]` 与 `[FPS] Display` 的 `source RX` 是否一致；
- `[FPS] Display` 的 `presented` 与 `max display gap` 是否解释体感；
- `missing`、本地队列、反压时间和 HighGUI 最大耗时是否出现异常；
- 插值轮廓是否比基础版清晰；
- 插值画面是否仍有双边或拖影；
- 实际显示是否接近原始帧数的 20 倍；
- CPU 占用是否影响上游重建。

若轮廓清晰度改善但仍有双影，下一步优先对照双向光流；若只是轻微软化，再考虑增加
可配置锐化，而不是继续提高光流分辨率。
