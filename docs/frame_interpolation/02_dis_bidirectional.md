# 阶段 2：双向 DIS

状态：待阶段 1 实测后实施，当前不可调用。

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

两张图分别使用各自方向的流场变形到中间时刻，再进行融合。增加前后向一致性检查，
对不一致区域降低错误一侧的融合权重。

## 修改范围

仍然只修改：

- `src/visualization/include/visualization/dis_frame_interpolator.hpp`
- `src/visualization/src/dis_frame_interpolator.cpp`
- 新增阶段 2 YAML 和 launch

ROS 消息、重建和其他节点保持不变。

## 预期调用

阶段 2 完成后将提供独立入口：

```bash
ros2 launch visualization v_dis_bidirectional.launch.py
```

在对应 launch 文件提交前不要使用此命令。

## 计算开销

- 每对真实帧计算两次低分辨率 DIS；
- 每张中间帧仍以两次 Warp 和一次融合为主；
- 一致性检查增加少量逐像素计算；
- 相较阶段 1，光流估计部分约增加一倍。

## 验收重点

- 阶段 1 中的边缘重影是否减少；
- CPU 增量是否影响上游重建；
- 两秒内能否完成双向光流和中间帧生成；
- 遮挡区域是否仍有需要深度信息解决的问题。
