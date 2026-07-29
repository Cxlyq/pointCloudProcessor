# 阶段 1：单向低分辨率 DIS

状态：已实现，可试用。

## 目标

保持现有流水线不变。`visualization_node` 接收相邻两个 `O3DMesh` 后，使用同一相机
将它们渲染为 `I0` 和 `I1`，在 1/4 分辨率上计算一次单向 DIS 光流，再复用这份
光流生成中间画面。

默认参数：

```text
每对真实帧插入：4 张虚拟帧
输出帧数倍率：约 5 倍
播放间隔：实际真实帧间隔 / 5
光流缩放：1/4
DIS 预设：ULTRAFAST
光流方向：I0 -> I1，一次
```

光流仍只在可视化包内部处理。`visualization_node` 只把真实渲染帧通过
`sensor_msgs/Image` 发送给独立显示进程；中间画面在显示进程本地生成和消费。

## 实现文件

- `src/visualization/src/visualization_node.cpp`
  - 捕获 Open3D RGB 渲染画面；
  - 锁定配置中的相机；
  - 插帧模式下隐藏 Open3D 源渲染窗口，只显示 OpenCV 插值窗口；
  - 仅在真实网格到达时渲染一次源画面，避免持续重绘阻塞插值播放。
- `src/visualization/src/interpolation_display_node.cpp`
  - 在独立 ROS 2 进程的主线程创建和更新 HighGUI 窗口；
  - ROS 接收线程与 HighGUI 主线程分离，DDS 只传真实源帧；
  - 本地 worker 生成完整插值序列，主线程按顺序逐帧消费；
  - 默认使用主线程 `waitKey(1)` 并记录 HighGUI 阻塞耗时。
- `src/visualization/src/dis_frame_interpolator.cpp`
  - 后台线程计算低分辨率 DIS；
  - 将光流放大并按实际宽高比例修正位移；
  - 默认生成 4 张中间帧；
  - 根据真实帧到达间隔调度中间帧，迟到时也逐帧播放而不跳过。
- `src/visualization/config/v_dis_ultrafast_config.yaml`
  - 第一阶段独立参数。
- `src/visualization/launch/v_dis_ultrafast.launch.py`
  - 第一阶段独立入口。

## 环境准备

目标 ROS 2 机器需要 OpenCV 开发包：

```bash
sudo apt update
sudo apt install libopencv-dev
```

Open3D、ROS 2 和本项目原有依赖仍按现有环境准备。

## 构建

在工作空间根目录执行：

```bash
colcon build --packages-up-to visualization
source install/setup.bash
```

如果修改了 YAML，而工作空间没有使用 `--symlink-install`，需要重新构建或重新安装
可视化包。

## 调用

其他流水线节点仍按原来的方式启动。最后启动第一阶段可视化：

```bash
ros2 launch visualization v_dis_ultrafast.launch.py
```

launch 会启动生成节点和独立显示节点，默认只出现一个可见窗口：

`Real-time Render - Front View - DIS x5`：延迟播放的插值画面。

Open3D 窗口仍作为源画面的渲染器存在，但默认隐藏，而且只在新网格到达时渲染一次。
需要排查源画面捕获时，可将 `interpolation_show_source_window` 临时改为 `true`。

首张真实网格到达时只能显示静态画面；第二张网格到达并完成光流计算后，才会开始
播放第一对插值结果。这是预期行为。

## 参数

编辑 `src/visualization/config/v_dis_ultrafast_config.yaml`：

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `interpolation_enabled` | `true` | 开启插帧窗口 |
| `interpolation_intermediate_frames` | `4` | 每两个真实帧之间插入的虚拟帧数 |
| `interpolation_flow_scale` | `0.25` | DIS 计算分辨率 |
| `interpolation_dis_preset` | `ultrafast` | `ultrafast`、`fast` 或 `medium` |
| `interpolation_bidirectional_flow` | `false` | 保持阶段 1 的单向光流 |
| `interpolation_flow_consistency_mask` | `false` | 仅双向流可用；阶段 1 保持关闭 |
| `interpolation_timing_source` | `message_stamp` | 使用发布端源帧时间戳计算播放步长 |
| `interpolation_pending_pair_capacity` | `3` | 待生成源帧对容量；满时阻塞反压，不合并帧对 |
| `interpolation_ready_sequence_capacity` | `2` | 已生成完整序列容量；满时等待显示端，不删除序列 |
| `interpolation_highgui_event_mode` | `wait_key` | 在显示主线程通过 `waitKey(1)` 处理并测量 HighGUI 事件 |
| `interpolation_lock_camera` | `true` | 每张真实网格都恢复相同相机 |
| `interpolation_show_source_window` | `false` | 是否显示仅用于调试的 Open3D 源窗口 |

若计算仍然太慢，先把 `interpolation_flow_scale` 降为 `0.125`。若速度足够但轮廓
抖动明显，可先将预设改为 `fast`，再决定是否进入阶段 2。

若将 `interpolation_intermediate_frames` 改为 `N`，长期帧数倍率约为 `N + 1`。
例如原始帧率为 0.5 FPS、`N = 4` 时，输出约为 2.5 FPS；原始帧率为 1 FPS 时，
输出约为 5 FPS。光流批量生成耗时不等于播放 FPS。

## 对照运行

关闭当前可视化节点后，可使用原始启动方式进行对照：

```bash
ros2 launch visualization v_vis.launch.py
```

原始配置没有启用插帧，因此不会创建 OpenCV 插值窗口。

## 试用检查项

- 日志是否显示 `Single-direction DIS interpolation enabled`；
- 第二张真实网格到达后是否开始平滑播放；
- `[GEN]` 是否显示生成 4 张中间显示帧及其总耗时；
- `[SRC-TX]` 与 `[FPS] Display` 的 `source RX` 是否一致；
- `presented` 是否接近 `source RX × 5`；
- `max display gap`、`missing`、本地队列、反压时间和 HighGUI 耗时是否解释体感；
- 实际显示帧数是否约为原始方式的 5 倍；
- 轮廓是否出现明显双影、拉伸或反方向移动；
- 光流计算期间 Open3D 和 ROS 是否仍能响应；
- CPU 占用是否影响上游重建速度；
- 两张真实网格之间是否存在明显黑边。

## 已知限制

- 单向光流的反向部分使用近似，不处理真实遮挡；
- 生成的是二维画面，不是中间三维网格；
- 用户修改相机后，下一个真实帧到达时会恢复配置相机；
- 原始 `v_vis.launch.py` 不启用插帧，相机仅在第一帧设置，不受
  `interpolation_lock_camera` 影响；
- 中间帧在后台预生成，计算完成前会保持上一张已显示画面。
- 完整中间帧序列进入 ready 队列后才会唤醒输出调度线程；显示迟到会拉长播放时间，
  不会跳过已经生成的显示帧。
- HighGUI 只在 `interpolation_display_node` 的主线程调用；ROS 回调在独立线程接收，
  避免 `waitKey()` 阻塞时 DDS 只能保留最新一帧。
