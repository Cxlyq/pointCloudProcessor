# 阶段 2：双向 DIS

状态：已实现，可试用。

## 进入条件

只有阶段 1 出现以下现象时再实现：

- 目标边缘存在明显双影；
- 前景和背景交界处向错误方向拉伸；
- 从画面中出现或消失的区域产生大面积拖影。

## 方案

每对真实渲染图分别计算：

```text
F01 = DIS(I0, I1)
F10 = DIS(I1, I0)
```

两张图分别使用各自方向的流场变形到中间时刻，再进行融合。当前第一版先使用真实
双向流替代阶段 1 的反向近似，并可选计算前后向一致性掩码。当前双向配置继续启用该
掩码；本次连续性修复不改变这一既有画质策略。

该功能只在 `interpolation_flow_consistency_mask: true` 时执行额外的全分辨率检查，
单向和 x20 配置默认关闭，不增加它们的计算量。掩码必须与
`interpolation_bidirectional_flow: true` 同时使用，否则节点会在启动时拒绝配置。

## 修改范围

仍然只修改：

- `src/visualization/include/visualization/dis_frame_interpolator.hpp`
- `src/visualization/src/dis_frame_interpolator.cpp`
- `src/visualization/src/visualization_node.cpp`
- `src/visualization/src/interpolation_display_node.cpp`
- `src/visualization/config/v_dis_bidirectional_config.yaml`
- `src/visualization/launch/v_dis_bidirectional.launch.py`

ROS 消息、重建和其他节点保持不变。

## 构建与调用

重新构建并加载工作空间：

```bash
colcon build --packages-up-to visualization
source install/setup.bash
```

其他流水线节点仍分别使用原来的 launch。可视化节点改用：

```bash
ros2 launch visualization v_dis_bidirectional.launch.py
```

默认只显示一个双向 DIS 插值窗口，Open3D 源渲染窗口保持隐藏。

默认同样在每两个真实帧之间插入 4 张虚拟帧，并根据真实帧间隔自动确定播放步长。
仍使用 1/4 分辨率和 `ULTRAFAST`，用于先比较双向流与单向流的差异。若生成时间明显
低于真实帧间隔但轮廓仍不稳定，可把配置改成：

```yaml
interpolation_flow_scale: 0.5
interpolation_dis_preset: "fast"
```

该入口使用独立的 `interpolation_display_node` 显示进程。发布与订阅均使用
`reliable + KeepAll`，显示回调将收到的帧按序放入深度 10 的 FIFO；队列满时向发布端
反压，而不是覆盖旧帧。ROS 接收线程不会被
HighGUI 的事件处理阻塞。输出调度迟到时一次只提交当前帧，并从实际提交时刻安排
下一帧，不再跳过中间帧或重置到最新真实帧。

## 计算开销

- 每对真实帧计算两次低分辨率 DIS；
- 每张中间帧仍以两次 Warp 和一次融合为主；
- 启用一致性掩码后，每对真实帧增加两次流场往返检查，每张中间帧增加两次单通道
  掩码 Warp；
- 同一分辨率下，DIS 实例、坐标网格和 Warp 临时缓冲会在线程内复用，避免每对真实
  帧重复构造和分配；分辨率变化时会重建 DIS 实例，输出帧缓冲始终独立持有，不能与
  缓存复用；
- 相较阶段 1，双向光流估计部分约增加一倍，一致性掩码还会增加一部分全分辨率开销。

## 验收重点

- 阶段 1 中的边缘重影是否减少；
- CPU 增量是否影响上游重建；
- 双向光流和 4 张中间帧能否在下一个真实帧到达前完成；
- 输出帧数是否约为原始帧数的 5 倍；
- 开启/关闭 `interpolation_flow_consistency_mask` 时，遮挡边界重影是否确实改善；
- 遮挡区域是否仍有需要深度信息解决的问题。

当前仍使用双线性 Warp，且一致性掩码只在“仅一端可信”时替换线性融合。若启用后
遮挡边界仍模糊，再进入带软权重的遮挡处理或阶段 3，而不是继续提高输出帧数。
