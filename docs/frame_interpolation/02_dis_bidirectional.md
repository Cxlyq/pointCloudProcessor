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
双向流替代阶段 1 的反向近似，不加入前后向一致性和遮挡修复，以便先判断双向流本身
能否降低轮廓重影。

## 修改范围

仍然只修改：

- `src/visualization/include/visualization/dis_frame_interpolator.hpp`
- `src/visualization/src/dis_frame_interpolator.cpp`
- `src/visualization/src/visualization_node.cpp`
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

默认仍使用 1/4 分辨率和 `ULTRAFAST`，用于先比较双向流与单向流的差异。若生成时间
明显低于真实帧间隔但轮廓仍不稳定，可把配置改成：

```yaml
interpolation_flow_scale: 0.5
interpolation_dis_preset: "fast"
```

## 计算开销

- 每对真实帧计算两次低分辨率 DIS；
- 每张中间帧仍以两次 Warp 和一次融合为主；
- 相较阶段 1，光流估计部分约增加一倍。

## 验收重点

- 阶段 1 中的边缘重影是否减少；
- CPU 增量是否影响上游重建；
- 两秒内能否完成双向光流和中间帧生成；
- 遮挡区域是否仍有需要深度信息解决的问题。

当前仍使用双线性 Warp 和线性融合。若双向流后遮挡边界仍模糊，再进入前后向一致性
遮挡权重或阶段 3，而不是继续提高输出帧数。
