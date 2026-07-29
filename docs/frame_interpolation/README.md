# 可视化插帧方案

## 当前运行方案

当前正式测试入口是双向 DIS ×5：

```bash
ros2 launch visualization v_dis_bidirectional.launch.py
```

每两个真实渲染帧之间生成 4 张中间帧，长期显示帧数约为真实帧数的
5 倍。现有双向光流一致性掩码保持启用，本次连续性修复不改变该画质策略。

修改范围仍限定在 `src/visualization`。点云输入、地面分割、聚类、重建、
网格消息以及其他节点均不修改。

## 运行结构

插帧模式仍使用两个 ROS 2 进程，但大尺寸图像只跨 DDS 传输一次：

```text
visualization_node
  Open3D 更新、渲染、截取真实帧
        |
        | reliable KeepAll
        | interpolation_source_frames/<camera_id>
        v
interpolation_display_node
  ROS 接收线程
        |
        v
  本地 DIS worker：A→B、B→C、C→D
        |
        v
  显示主线程：按顺序逐帧 imshow + HighGUI 事件处理
```

旧实现会把 ×5 后的每一张 BGR 图像都经 DDS 发送，再由显示进程 FIFO
消费；最终显示端较慢时，这条链路会积压。新实现只发送真实帧，DIS
中间帧保留在最终显示进程内，减少约 80% 的图像 DDS 流量，也让生成
调度和最终显示消费共用同一条有序时间线。

## 连续性与过载策略

优先级是：

1. 不丢真实帧对，不丢已经生成的中间帧；
2. 尽量保持实时；
3. 延迟最低优先。

待生成队列和完整序列队列都是有界且严格有序的：

```yaml
interpolation_pending_pair_capacity: 3
interpolation_ready_sequence_capacity: 2
```

队列满时，ROS 源帧回调会等待可用位置。代码不会再把 `B→C` 合并成
`B→D`，也不会删除完整的中间帧序列。显示端落后时，每次仍只消费当前
应显示的一帧，并从实际消费时刻安排下一帧；不会为了追赶实时时钟跳过
中间帧。代价是延迟和 DDS 反压可能增长，这正是当前明确选择的过载行为。

源帧消息由 `visualization_node` 在发布前写入单调序号和时间戳。显示端
默认使用消息时间戳计算插帧间隔，因此即使 ROS 回调因反压等待，播放
步长也不会被这段等待时间错误拉长。

## HighGUI

默认配置为：

```yaml
interpolation_highgui_event_mode: "wait_key"
```

窗口创建、`imshow()` 和事件处理全部位于显示进程主线程。一次实测中，
`pollKey()` 每轮耗时达到 700～1100 ms，而 `imshow()` 只有 1～4 ms，
因此正式 ×5 配置改为调用 `waitKey(1)`。它仍会处理 HighGUI 事件并纳入
显示边界计时，但不会沿用这次运行中表现异常的 `pollKey()` 路径。

`poll_key` 保留为诊断对照。也可以显式配置 `start_window_thread`；若当前
HighGUI 后端不支持事件线程，程序会自动回退到主线程 `waitKey(1)`。启动
日志和 `[PIPE]` 会分别报告请求模式与实际生效模式。

## 日志

日志按 5 秒窗口输出，重点保留以下几类：

- `[SRC-TX]`：真实渲染帧发布速率、渲染/截取、图像编码、DDS
  `publish()` 调用耗时、订阅者数量和被新网格覆盖的数量。
- `[GEN]`：每个真实帧对的双向 DIS 生成耗时及源帧序号，例如
  `source #12->#13`。
- `[FPS]`：显示进程实际收到的真实帧率和完成
  `imshow + event pump` 边界的帧率；同时报告 DDS 源序号缺失、
  乱序和本地显示序列断裂。
- `[PIPE]`：DDS 传输延迟、源帧提交耗时、最新源帧年龄、
  `imshow`/事件处理耗时、pending/ready/active 队列、worker 状态、
  调度迟到以及源端反压等待。
- `[FLOW]`：有序提交等待超过 100 ms。它表示主动反压，不表示丢帧。
- `[HIGHGUI]`：`imshow` 或事件处理超过 100 ms。

`presented` 的边界是应用完成 `imshow()` 和本轮 HighGUI 事件处理。
OpenCV 没有跨平台的“显示器已经物理扫描此帧”回调，因此该数值是应用
能获得的最接近最终显示端的真实统计，但不等同于显示器扫描时刻。

正常的双向 ×5 运行应满足：

```text
[SRC-TX] published ≈ [FPS] source RX
[FPS] presented ≈ source RX × 5
source missing = 0
out-of-order = 0
display sequence breaks = 0
source backpressure = 0（正常负载）
[PIPE] mode main-thread-waitKey(1)
```

如果 `source missing > 0`，帧丢在发布之后、显示回调之前，应继续检查
DDS。若缺失和序列断裂均为 0，但 `imshow` 或 event 的最大耗时达到
数百毫秒、`presented` 明显低于目标，则最终瓶颈仍在 HighGUI。若
pending/ready 长期满且 `source backpressure` 为 `ACTIVE`，说明最终
消费能力低于输入需求；此时帧仍保持连续，但端到端延迟会继续增长。
本轮修复验证时，还应确认 event 平均/最大耗时不再维持在
700～1100 ms，且 source backlog 和 newest-source age 不再持续增长。

## 构建与测试

```bash
colcon build --packages-select visualization \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

colcon test --packages-select visualization
colcon test-result --verbose
```

运行双向 ×5 后，保持与之前相同的场景运动至少 30～60 秒，并保存：

1. 启动阶段全部日志；
2. 连续 3 组 `[SRC-TX]`、`[FPS]` 和 `[PIPE]`；
3. 期间全部 `[GEN]`、`[FLOW]`、`[HIGHGUI]` 和警告；
4. 主观现象：是否仍跳变、是否冻结、延迟是否随时间持续增加。

## 其他入口

| 入口 | 中间帧 | 光流 | 用途 |
| --- | ---: | --- | --- |
| `v_vis.launch.py` | 0 | 无 | 原始 Open3D 显示对照 |
| `v_dis_bidirectional.launch.py` | 4 | 双向 + 一致性掩码 | 当前统一使用的 ×5 版本 |
| `v_dis_ultrafast.launch.py` | 4 | 单向 | 保留的历史低开销对照 |
| `v_dis_quality_x20.launch.py` | 19 | 单向 | 历史压力测试，不作为当前运行方案 |

不要同时启动多个可视化 launch。需要对比时，先停止当前可视化进程再
切换入口。

## 方案文档

- [单向低分辨率 DIS](01_dis_ultrafast_single_direction.md)
- [单向 DIS 清晰度 ×20 压力测试](01b_dis_quality_x20.md)
- [双向 DIS](02_dis_bidirectional.md)
- [深度边缘辅助 DIS](03_dis_depth_edge_guided.md)
- [全局仿射插帧](04_global_affine.md)
- [全局预对齐加 DIS](05_global_alignment_plus_dis.md)
- [延后的 RIFE 方案](90_rife_deferred.md)
