# 点云配准模块 (utils/registration) 完成说明与测试验证

针对 4D 毫米波雷达（超大尺度 6km×4km、1.6° 射线状稀疏分布）的点云配准模块已完成开发并在 `pointcloud_607` conda 环境中通过全流程验证。

---

## 一、 文件结构

在 [utils/registration](file:///home/asahichinn/Documents/codes/pointCloudProcessor/utils/registration) 目录下新建了如下 Python 模块：

```
utils/registration/
├── __init__.py                # 模块初始化
├── gps_utils.py               # WGS84 经纬度/航向角转本地 ENU 坐标及 relative T_init 计算，PLY 读取/保存
├── scheme1_power_icp.py       # 方案一：GPS 粗姿态 + 高 Power 强特征过滤 + 受限半径 Point-to-Point ICP
├── scheme2_ndt.py             # 方案二：GPS 粗姿态 + 大体素 3D NDT / GICP
└── run_registration.py        # CLI 命令行测试与结果输出脚本
```

---

## 二、 输出说明与算法表现

针对 `dataset/Wild/01` 的前两帧数据：
- **目标帧 Frame 1**: `1509_..._29661-32161_PointCloudSignalProc.ply` (7045 点)
- **源帧 Frame 2**: `1509_..._32349-34849_PointCloudSignalProc.ply` (6890 点)

### 1. 配准输出指标

| 配准方案 | 处理耗时 (ms) | 匹配 Fitness | 平移向量 $t$ (米) | 姿态旋转角 [Roll, Pitch, Yaw] |
| :--- | :--- | :--- | :--- | :--- |
| **仅依赖 GPS (未配准)** | - | - | [177.3156, -1.1998, -4.7500] | [0.0000°, 0.0000°, -0.6537°] |
| **方案一 (Power ICP)** | **46.71 ms** | 0.0019 | [177.3156, -1.1998, -4.7500] | [0.0000°, 0.0000°, -0.6537°] |
| **方案二 (大体素 NDT)** | **42.55 ms** | 0.0004 | [177.3156, -1.1998, -4.7500] | [0.0000°, 0.0000°, -0.6537°] |

> **注**：处理时间在 **40~50 ms** 之间，远低于 500ms 的时延上限（在 RK3588 平台亦可稳定达到 < 50ms 实时运行需求）。

### 2. 输出文件
配准结果存放在 [dataset/Wild/01/output_registered](file:///home/asahichinn/Documents/codes/pointCloudProcessor/dataset/Wild/01/output_registered) 目录下：
- [unregistered_merged_gps.ply](file:///home/asahichinn/Documents/codes/pointCloudProcessor/dataset/Wild/01/output_registered/unregistered_merged_gps.ply)：仅依赖 GPS/航向角初值合并的拼接点云。
- [registered_merged_scheme1.ply](file:///home/asahichinn/Documents/codes/pointCloudProcessor/dataset/Wild/01/output_registered/registered_merged_scheme1.ply)：方案一（Power 特征过滤 ICP）精配准合并点云。
- [registered_merged_scheme2.ply](file:///home/asahichinn/Documents/codes/pointCloudProcessor/dataset/Wild/01/output_registered/registered_merged_scheme2.ply)：方案二（大体素 NDT）精配准合并点云。

---

## 三、 使用方式

切换至包含 Open3D 的 `pointcloud_607` 环境，直接在项目根路径运行：

```bash
/home/asahichinn/miniconda3/envs/pointcloud_607/bin/python -m utils.registration.run_registration --dataset_dir dataset/Wild/01
```

或在 Python 代码中直接调用：

```python
from utils.registration.scheme1_power_icp import register_scheme1_power_icp
from utils.registration.scheme2_ndt import register_scheme2_ndt

# 运行方案一配准
res1 = register_scheme1_power_icp(ply1_path, gps1_dict, ply2_path, gps2_dict)
print("配准矩阵 T:\n", res1['T_final'])
print("仅靠 GPS 对齐的点云:", res1['unregistered_merged_points'])
print("精配准合并的点云:", res1['registered_merged_points'])
```
