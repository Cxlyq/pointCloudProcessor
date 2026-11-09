# connection_test 节点说明文档

`connection_test` 是针对双板/多机分布式部署专门设计的网络链路与物理带宽基准测试功能包。在大规模点云处理流水线中，单帧三角网格（`O3DMesh`）的数据量通常可达数兆至数十兆字节。该功能包提供高频网格模拟发布源（`mesh_publisher`）与端到端统计接收器（`mesh_subscriber`），用于压测千兆/万兆板间以太网在真实 DDS 通信分片传输下的物理极限带宽、单向网络传输延迟以及实时传输帧率。

---

## 1. 节点概览与设计目标

| 属性 | 内容 |
| :--- | :--- |
| **功能包名称** | `connection_test` |
| **可执行文件** | 1. `mesh_publisher`<br>2. `mesh_subscriber` |
| **主 C++ 类** | 1. `MeshPublisher` ([mesh_publisher.cpp](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/connection_test/src/mesh_publisher.cpp))<br>2. `MeshSubscriber` ([mesh_subscriber.cpp](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/connection_test/src/mesh_subscriber.cpp)) |
| **核心依赖** | ROS 2 (`rclcpp`, `std_msgs`), `pc_msgs` |

### 核心设计目标：
1. **多机跨网卡物理带宽压力测试**：用于在分布式部署（如开发板 1 负责白模生成，开发板 2 负责配准渲染）上线前，测试板载网卡与交换机在满载大包传输下的实际吞吐能力。
2. **纳秒级单向网络传输延迟评估**：发布端注入高精度纳秒系统时钟（`header.stamp = this->now()`），接收端根据到达时间直接计算纯传输耗时，剔除算法干扰。
3. **真实负载模拟**：预填包含 4000 个三角面（12000 个顶点索引及法线、色彩）的标准真实负载，规避空包虚高测试。
4. **确定性 QoS 传输策略**：采用 `KeepLast(5)`、`reliable()`、`durability_volatile()` 组合，模拟生产环境下大包分片重组的最严苛工况。

---

## 2. 核心机制与架构

### 2.1 整体架构图

```
      [计算节点 A / 开发板 1]                             [计算节点 B / 开发板 2]
 ┌───────────────────────────────┐                  ┌───────────────────────────────┐
 │        mesh_publisher         │                  │        mesh_subscriber        │
 │                               │                  │                               │
 │  * 预构建 4000 面 O3DMesh 模板 │                  │  * 纳秒时间差延迟计算         │
 │  * 高频恒速压测线程           │                  │  * 字节负载累加统计           │
 │  * 注入当前系统时间戳         │                  │  * 周期性输出吞吐与带宽报告   │
 └───────────────┬───────────────┘                  └───────────────▲───────────────┘
                 │                                                  │
                 └───────────────► /mesh_topic ─────────────────────┘
                             (pc_msgs/msg/O3DMesh)
                           以太网物理链路 (DDS 分片传输)
```

### 2.2 性能测算公式
接收端以 1.0 秒为周期统计窗口：
* **实时传输帧率 (FPS)**：
  $$\text{FPS} = \frac{\Delta \text{Frames}}{\Delta t_{\text{window}}}$$
* **平均单向传输延迟 (Latency)**：
  $$\text{Avg Latency} = \frac{1}{N} \sum_{i=1}^N (t_{\text{recv}, i} - t_{\text{stamp}, i})$$
* **有效吞吐带宽负载 (Bandwidth)**：
  $$\text{Payload} = \sum (\text{bytes}_{\text{vertices}} + \text{bytes}_{\text{triangles}} + \text{bytes}_{\text{normals}} + \text{bytes}_{\text{colors}})$$
  $$\text{Throughput (MB/s)} = \frac{\text{Payload} / 1024^2}{\Delta t_{\text{window}}}$$

---

## 3. 通信接口

### 3.1 话题定义 (Topics)

| 节点 | 话题名称 | 通信方向 | 消息类型 | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| `mesh_publisher` | `mesh_topic` | 发布 (Publish) | [pc_msgs/msg/O3DMesh](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/O3DMesh.msg) | 标准压力测试三角网格 |
| `mesh_subscriber` | `mesh_topic` | 订阅 (Subscribe) | [pc_msgs/msg/O3DMesh](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/O3DMesh.msg) | 接收并统计吞吐延迟 |

---

## 4. 统计日志输出示例

当接收端处于正常压测运行状态时，控制台每秒输出一次网络指标快报：

```text
[mesh_subscriber-2] [INFO] [吞吐统计] 帧率: 49.8 FPS | 平均网络延迟: 1.215 ms | 带宽负载: 14.35 MB/s
```

若发送端停止或网线物理断开，输出警告：
```text
[mesh_subscriber-2] [WARN] 等待接收数据中... (当前尚未收到 mesh 数据帧)
```

---

## 5. 启动与使用命令参考

### 5.1 单机跨进程环回基准测试
在同一台机器的两个终端中分别执行：

```bash
# 终端 1：启动接收器并监控统计
ros2 run connection_test mesh_subscriber

# 终端 2：启动发布器开始压测
ros2 run connection_test mesh_publisher
```

### 5.2 双板 / 分布式跨机压测
假设两块计算板通过以太网直连，并处于同一 `ROS_DOMAIN_ID`：

```bash
# 开发板 2 (接收端)：
export ROS_DOMAIN_ID=42
ros2 run connection_test mesh_subscriber

# 开发板 1 (发送端)：
export ROS_DOMAIN_ID=42
ros2 run connection_test mesh_publisher
```

可通过调整 `mesh_publisher.cpp` 中的 `std::this_thread::sleep_for(std::chrono::microseconds(20000))` 控制发送节奏，如调至 500 微秒或 100 微秒，测试网卡与操作系统网络栈物理丢包临界点。

---

## 6. 文件结构与索引

```
src/connection_test/
├── CMakeLists.txt              # 构建脚本 (生成两个独立测试执行文件)
├── package.xml                 # ROS 2 包定义
└── src/
    ├── mesh_publisher.cpp      # 高频压测网格发布者实现
    └── mesh_subscriber.cpp     # 吞吐量、延迟与带宽统计接收者实现
```
