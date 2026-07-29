# 阶段 2：双向 DIS ×5

状态：当前统一使用的可视化插帧方案。

## 算法

对每对真实渲染图计算两个方向的光流：

```text
F01 = DIS(I0, I1)
F10 = DIS(I1, I0)
```

两张端点图分别使用对应方向的流场变形到中间时刻，再线性融合。当前
配置同时启用前后向一致性掩码：

```yaml
interpolation_intermediate_frames: 4
interpolation_flow_scale: 0.25
interpolation_dis_preset: "ultrafast"
interpolation_bidirectional_flow: true
interpolation_flow_consistency_mask: true
```

一致性掩码属于既有画质策略，本次连续性修改不改变其行为。

## 进程边界

`visualization_node` 只负责 Open3D 网格更新、真实帧渲染和源图发布。
`interpolation_display_node` 接收真实帧后，在本进程的 worker 中完成
双向 DIS，并由显示主线程按顺序消费全部中间帧。

DDS 话题为：

```text
interpolation_source_frames/<camera_id>
```

发布和订阅均使用 `reliable + KeepAll`。×5 后的中间 BGR 图不再经过
DDS，因此避免让大尺寸图像发布链和显示 FIFO 成为额外瓶颈。

## 连续性

本地队列使用严格顺序和阻塞反压：

```yaml
interpolation_pending_pair_capacity: 3
interpolation_ready_sequence_capacity: 2
```

当 pending 已满，新的源帧提交等待 worker 腾出位置；不会把多个相邻
源帧对合并。当 ready 已满，worker 等待显示端完成前面的序列；不会
删除完整序列。显示调度即使迟到也只取下一张帧，不通过跳帧追赶。

因此过载时的可见结果应为播放变慢、延迟和反压上升，而不是从一段中间
帧突然跳到后续真实帧。

## 构建与启动

```bash
colcon build --packages-select visualization \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch visualization v_dis_bidirectional.launch.py
```

完整的日志解释和现场测试要求见
[可视化插帧方案说明](README.md)。

## 验收重点

- `source missing = 0`、`out-of-order = 0`；
- `display sequence breaks = 0`；
- `[GEN]` 中源帧对连续，例如 `#12->#13`、`#13->#14`；
- 长期 `presented ≈ source RX × 5`；
- 正常负载下 pending/ready 不长期占满；
- `imshow` 和 event 耗时没有持续数百毫秒；
- 主观观察无整段跳变，且延迟不持续无界增长。
