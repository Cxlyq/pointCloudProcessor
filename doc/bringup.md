# bringup 节点说明文档

`bringup` 是点云处理流水线的总起主控功能包。它负责集中管理整个处理流水线中各功能节点的生命周期，提供多节点参数化拉起、自定义顺序编排、进程间输出过滤与日志等级控制，以及基于端到端处理结果的系统全局运行效率与帧率（FPS）监控。

---

## 1. 节点概览与设计目标

| 属性 | 内容 |
| :--- | :--- |
| **功能包名称** | `bringup` |
| **可执行文件** | `bringup_node` |
| **主 C++ 类** | `BringupNode` ([bringup_node.cpp](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/bringup/src/bringup_node.cpp)) |
| **核心依赖** | ROS 2 (`rclcpp`), `pc_msgs` |

### 核心设计目标：
1. **多机/双板分布式部署支持**：支持将数据源到白模重建 (`io_handlers -> ground_segmentation -> clustering -> reconstruction`) 与流式配准到可视化 (`flow_alignment -> visualization`) 拆分部署在不同计算板上，通过参数开关灵活拉起各自负责的节点集合。
2. **启动顺序自适应与优先级编排**：依据节点依赖关系，支持由下至上（下游消费者先就绪、上游数据源后启动）或任意自定义顺序拉起节点，避免数据丢失。
3. **终端日志分流与等级过滤**：为每个子进程建立独立管道读取机制，防止终端各节点输出互相穿插杂乱，支持按节点开关输出，并支持按日志等级（`INFO` / `WARN` / `ERROR` / `NONE`）过滤噪音。
4. **系统全局效率与端到端延迟统计**：集中统计整个流水线的处理帧率、端到端延迟与吞吐能力，实时反馈系统运行瓶颈。

---

## 2. 核心机制与架构

### 2.1 整体架构图

```
                       ┌──────────────────────────────────────────────┐
                       │                 bringup_node                 │
                       │                                              │
                       │  ┌────────────────────────────────────────┐  │
                       │  │         优先级排序 / 状态机编排          │  │
                       │  └──────────────────┬─────────────────────┘  │
                       │                     │                        │
       ┌───────────────┼─────────────────────┼────────────────────────┼────────────────┐
       ▼ (fork/pipe)   ▼ (fork/pipe)         ▼ (fork/pipe)            ▼ (fork/pipe)    ▼ (fork/pipe)
┌─────────────┐ ┌───────────────────┐ ┌──────────────┐      ┌────────────────┐ ┌─────────────┐
│ io_handlers │ │ground_segmentation│ │  clustering  │ ...  │ flow_alignment │ │visualization│
└──────┬──────┘ └─────────┬─────────┘ └──────┬───────┘      └───────┬────────┘ └─────────────┘
       │ [点云]           │ [非地面点]        │ [聚类结果]           │ [对齐网格]
       └──────────────────┴──────────────────┴──────────────────────┘
                                      │
                                      ▼ 监听末端主题 (如 /flow/aligned_mesh)
                       ┌──────────────────────────────────────────────┐
                       │            效率与帧率监控模块 (FPS)           │
                       │  * 瞬时/周期平均 FPS                          │
                       │  * 端到端总延迟 (now - header.stamp)          │
                       │  * 信号捕捉 ([RX] Accepted mesh)             │
                       └──────────────────────────────────────────────┘
```

### 2.2 节点拉起与优雅退出
* **独立进程组隔离**：对每个拉起的节点调用 `setpgid(0, 0)` 创建独立进程组，避免因单个子节点异常退出波及主进程，同时便于信号组发。
* **优雅逆序关闭**：支持 `shutdown_in_reverse` 参数。在收到 `Ctrl+C` 关闭信号时，可逆序逐个向子进程组发送 `SIGINT`，若超过等待时间（默认 3 秒）未退出则升级为 `SIGKILL` 强制回收，确保资源不泄漏。

### 2.3 管道化终端输出与日志过滤
* **静音重定向**：对配置为 `show_output: false` 或 `log_level: "NONE"` 的节点，子进程直接将其 `stdout` 和 `stderr` 重定向至 `/dev/null`，零性能损耗且不污染终端屏幕。
* **行缓冲与等级过滤**：子进程启用 `PYTHONUNBUFFERED=1` 保证行级实时性；父进程通过后台独立线程读取每行日志并判定：
  * **`WARN` 等级**：过滤掉所有普通 `[INFO]` 与 `[DEBUG]` 日志，只输出 `[WARN]`、`[ERROR]`、`[FATAL]` 以及代码异常堆栈（`Exception`, `Traceback`）。
  * **`ERROR` 等级**：过滤掉普通 `[INFO]`、`[DEBUG]` 与 `[WARN]`，仅输出严重报错。
  * **颜色前缀隔离**：多线程在输出前获取互斥锁 `stdout_mutex_`，并在行首添加彩色节点名前缀（如 `[visualization]`、`[reconstruction]`），彻底杜绝字符交叉穿插。

---

## 3. 系统运行效率与全流程耗时瓶颈排查 (Bottleneck Analysis)

### 3.1 周期统计与耗时分析输出示例

当启用效率监控时，`bringup` 会在设定的周期（默认每 3.0 秒）在终端输出效率摘要与自动化的流水线耗时瓶颈剖析：

```text
[bringup_node-1] [SYSTEM EFFICIENCY] [Signal: visualization] Window: 3.0s | Throughput: 15.20 FPS (Instant: 15.02 FPS) | Frames: 45 (Total: 135)
┌──────────────────────── Pipeline Stage Latency Breakdown ────────────────────────┐
│  io_handlers          | Frames:   45 | Avg:    0.0 ms | Max:    0.0 ms | Last:    0.0 ms                 │
│  ground_segmentation  | Frames:   45 | Avg:   32.4 ms | Max:   41.0 ms | Last:   30.2 ms                 │
│  clustering           | Frames:   45 | Avg:   65.1 ms | Max:   78.6 ms | Last:   62.8 ms                 │
│  reconstruction       | Frames:   45 | Avg:  212.0 ms | Max:  260.5 ms | Last:  198.4 ms <-- [BOTTLENECK] │
│  flow_alignment       | Frames:   45 | Avg:  110.5 ms | Max:  128.0 ms | Last:  105.2 ms                 │
│  visualization        | Frames:   45 | Avg:    8.5 ms | Max:   14.2 ms | Last:    7.9 ms                 │
└──────────────────────────────────────────────────────────────────────────────────┘
```

或者在开启了 Topic 订阅并携带原始时间戳时：
```text
[bringup_node-1] [SYSTEM EFFICIENCY] [Topic: /flow/aligned_mesh] Window: 3.0s | Throughput: 15.20 FPS (Instant: 15.02 FPS) | E2E Latency: 46.8 ms [min: 41.2, max: 55.4] | Frames: 45 (Total: 135)
```

每个数字与字段的具体含义如下表所示：

| 字段名称 | 示例数值 | 单位 | 计算公式 / 来源 | 详细含义说明 |
| :--- | :--- | :--- | :--- | :--- |
| **`[bringup_node-1]`** | `1` | - | ROS 2 Launch 系统生成 | `bringup_node` 进程在本次 launch 中的唯一实例标识符。 |
| **`[Topic: ...]` / `[Signal: <node>]`** | 标签 | - | 统计来源模式标记 | 标识当前数据是由 ROS 2 Topic 订阅测算，还是由指定节点的标准输出日志信号（零 DDS 开销模式）测算。 |
| **`Window`** | `3.0s` | 秒 (`s`) | 由参数 `efficiency_report_interval_sec` 指定 | **统计滑动窗口时长**。表示该行日志中的所有吞吐率、帧数、延迟统计均基于过去这 3.0 秒内的样本计算得出。 |
| **`Throughput`** | `15.20 FPS` | 帧/秒 (`Hz`) | $\text{Frames in Window} \div \text{Window Elapsed Time}$ | **窗口平均吞吐帧率**。过去 3.0 秒内流水线单位时间能够完整输出交付给终端的平均帧率，直接反映系统的实际生产能力。 |
| **`Instant`** | `15.02 FPS` | 帧/秒 (`Hz`) | $1.0 \div (t_{\text{now}} - t_{\text{last}})$ | **实时瞬时帧率**。基于最新到达的一帧与前一帧之间的精确到达时间间隔倒数计算，用于快速观察单帧抖动或偶发卡顿。 |
| **`E2E Latency`** | `46.8 ms` | 毫秒 (`ms`) | $\frac{1}{N}\sum(t_{\text{recv}} - t_{\text{source\_stamp}})$ | **端到端平均处理延迟**（仅 Topic 模式提供）。数据从最上游传感器采样打上时间戳（`io_handlers` 的 `header.stamp`），历经地面分割、聚类、白模重建、流式配准，直到流水线最终输出的整体平均消耗时间。 |
| **`min`** | `41.2` | 毫秒 (`ms`) | $\min(t_{\text{recv}} - t_{\text{source\_stamp}})$ | **窗口内最小端到端延迟**。反映本统计周期内最轻计算负载或最佳工况下的单帧耗时。 |
| **`max`** | `55.4` | 毫秒 (`ms`) | $\max(t_{\text{recv}} - t_{\text{source\_stamp}})$ | **窗口内最大端到端延迟**。反映本统计周期内的最差耗时（例如突发点云量骤增、泊松重建深度增加等带来的计算峰值）。 |
| **`Frames`** | `45` | 帧 | $\sum_{\text{window}} 1$ | **当前统计周期内完成并接收到的有效帧总数**。如 3 秒内处理了 45 帧。 |
| **`Total`** | `135` | 帧 | $\sum_{\text{start}} 1$ | **自节点启动以来累计完成的总帧数**。持续递增，用于追踪系统长期稳定运行的无丢帧累计量。 |
| **`<-- [BOTTLENECK]`** | 标签 | - | 自动对比各节点平均处理耗时 | **流水线瓶颈自动标识**。自动标出平均处理时间最长的阶段，提示开发者针对该节点做算法或算力优化。 |

### 3.2 节点逐帧确认信号规范与耗时排查

为了排查整条点云流水线中究竟是哪一个节点耗时最长，所有 6 个节点均已加入标准化的逐帧确认信号：

| 节点名称 | 阶段确认标准日志格式 | 耗时统计逻辑 |
| :--- | :--- | :--- |
| **`io_handlers`** | `[STAGE-DONE] io_handlers: Frame %zu published \| File: '%s' \| Points: %zu \| Time: 0.00 ms` | 数据发布确认，耗时 0 ms |
| **`ground_segmentation`** | `[STAGE-DONE] ground_segmentation: Frame %zu processed \| Ground: %zu, Non-Ground: %zu \| Time: %.2f ms` | GMZ 地面点与非地面点滤波分割耗时 |
| **`clustering`** | `[STAGE-DONE] clustering: Frame %zu processed \| Input: %zu, Clusters: %d \| Time: %.2f ms` | 并行 DBSCAN 聚类算法耗时 |
| **`reconstruction`** | `[STAGE-DONE] reconstruction: Frame %zu processed \| Vertices: %zu, Triangles: %zu \| Time: %.2f ms` | 泊松地面重建 + OBB 点集外包网格生成耗时 |
| **`flow_alignment`** | `[STAGE-DONE] flow_alignment: Frame %zu processed \| Aligned vertices: %zu \| Time: %.2f ms` | ICP 流式全局刚体位姿配准耗时 |
| **`visualization`** | `[STAGE-DONE] visualization: Frame %zu received \| Vertices: %zu, Triangles: %zu \| Time: %.2f ms` | 末端渲染接收与网格解析耗时 |

`bringup` 会在后台静默解析所有启用的子节点的 `[STAGE-DONE]` 信号，提取其中的单帧耗时，并在统计周期结束时输出完整的 `Pipeline Stage Latency Breakdown` 耗时明细表。即便你在配置文件中将某个节点的 `show_output` 设为 `false`（终端静音），`bringup` 依然可以在后台提取该节点的性能耗时指标，做到**既不污染终端，又能掌控全局瓶颈**。

### 3.3 两种效率监控模式对比与选型建议

针对 `/flow/aligned_mesh` 单帧数据量极大（数兆到数十兆字节）的特点，`bringup` 支持两种互补模式：

1. **纯标准输出日志信号模式 (Zero-DDS Overhead，推荐)**：
   * **配置方法**：在 YAML 中设置 `efficiency_monitor_topic: ""`（留空），保留 `enable_efficiency_monitor: true`，并通过 `monitor_target_node` 指定监控目标节点（如 `"visualization"` 或 `"reconstruction"`）。
   * **工作原理**：`bringup` 不订阅任何网格话题，零额外网络传输与反序列化开销；直接通过管道读取目标节点每帧的 `[STAGE-DONE]` 确认信号，驱动帧率与累计帧计数。
   * **适用场景**：生产环境、大场景高分辨率网格实时显示、单机与双板部署。

2. **ROS 2 Topic 订阅模式**：
   * **配置方法**：在 YAML 中指定 `efficiency_monitor_topic: "/flow/aligned_mesh"`。
   * **工作原理**：`bringup` 订阅出口话题，能够精确获得消息包中的原始时间戳 `header.stamp`，进而计算端到端总延迟（E2E Latency）。
   * **适用场景**：调试阶段需精确测定流水线全链路计算延迟（`E2E Latency`）的测试基准。

> [!NOTE]
> 若某周期内目标节点未处理任何数据，输出将变为警告：
> `[SYSTEM EFFICIENCY] [Signal: visualization] No new frames received in the last 3.0s. Total: 135`
> 帮助开发者立即定位上游数据中断或堵塞问题。

---

## 4. 通信接口

### 4.1 订阅话题 (Subscribed Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `efficiency_monitor_topic` (配置指定) | [pc_msgs/msg/O3DMesh](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/O3DMesh.msg) | 监控流水线端到端吞吐率的网格主题（单机/开发板 2 默认 `/flow/aligned_mesh`；开发板 1 默认 `/reconstruction/white_mesh`） |

---

## 5. 参数配置说明

配置文件位于 [`src/bringup/config/`](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/bringup/config/) 目录下。

### 5.1 全局控制参数

| 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `launch_interval_sec` | `double` | `3.0` | 依次拉起相邻两个启用的子节点之间的时间间隔（单位：秒） |
| `shutdown_in_reverse` | `bool` | `false` | 退出时是否按拉起顺序的相反顺序逆向关闭节点 |
| `custom_launch_order` | `string[]` | `[]` | 可选显式拉起顺序。若非空则优先使用此列表；若为空则依各节点 `priority` 升序拉起 |
| `global_show_output` | `bool` | `true` | 全局终端输出总开关（设为 `false` 则全部子节点静音） |
| `global_log_level` | `string` | `"INFO"` | 全局日志过滤级别（`"INFO"`, `"WARN"`, `"ERROR"`, `"NONE"`） |
| `enable_color_prefix` | `bool` | `true` | 终端打印每行时是否添加彩色包名前缀 |
| `enable_efficiency_monitor` | `bool` | `true` | 是否启用系统全局效率与帧率统计 |
| `efficiency_monitor_topic` | `string` | `""` | 用于帧率与端到端统计的监控话题（留空 `""` 进入零 DDS 日志监听模式） |
| `monitor_target_node` | `string` | `"visualization"` | 零 DDS 模式下指定监听效率的目标节点（如 `"visualization"`、`"reconstruction"` 等） |
| `efficiency_report_interval_sec` | `double` | `3.0` | 统计结果的终端打印周期（秒） |
| `efficiency_log_signal_keyword` | `string` | `"[STAGE-DONE]"` | 识别阶段完成确认日志的关键字 |

### 5.2 各功能节点独立配置参数（以 `<pkg>` 为例）

每个功能节点（`visualization`, `flow_alignment`, `reconstruction`, `clustering`, `ground_segmentation`, `io_handlers`）均拥有一套正交参数：

| 参数模式 | 类型 | 示例 | 说明 |
| :--- | :--- | :--- | :--- |
| `<pkg>_enable` | `bool` | `reconstruction_enable: true` | 是否拉起该节点（`true` 拉起，`false` 跳过） |
| `<pkg>_priority` | `int` | `reconstruction_priority: 3` | 启动优先级数值（**数值越小越早拉起**） |
| `<pkg>_show_output` | `bool` | `reconstruction_show_output: true` | 是否在终端输出该节点内容（`false` 完全静音） |
| `<pkg>_log_level` | `string` | `reconstruction_log_level: ""` | 节点级日志过滤覆盖（留空则继承 `global_log_level`） |
| `<pkg>_launch` | `string` | `reconstruction_launch: "..."` | 该节点调用的启动文件名 |

---

## 6. 预设配置与多机部署方案

`bringup` 原生提供了三份适配不同工况的配置文件：

### 6.1 单机全量模式 ([bring_up.yaml](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/bringup/config/bring_up.yaml))
* 启用全部 6 个节点。
* 推荐启动优先级（消费者先就绪）：
  `visualization (1)` $\to$ `flow_alignment (2)` $\to$ `reconstruction (3)` $\to$ `clustering (4)` $\to$ `ground_segmentation (5)` $\to$ `io_handlers (6)`。
* 效率监控主题：`/flow/aligned_mesh`。

### 6.2 开发板 1 部署模式 ([bring_up_board1.yaml](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/bringup/config/bring_up_board1.yaml))
* **任务范围**：数据输入、地面分割、聚类、白模重建。
* **启用节点**：`reconstruction (1)` $\to$ `clustering (2)` $\to$ `ground_segmentation (3)` $\to$ `io_handlers (4)`。
* **禁用节点**：`flow_alignment (false)`、`visualization (false)`。
* **效率监控主题**：`/reconstruction/white_mesh`（统计开发板 1 的产出吞吐率）。

### 6.3 开发板 2 部署模式 ([bring_up_board2.yaml](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/bringup/config/bring_up_board2.yaml))
* **任务范围**：流式配准、三维渲染呈现。
* **启用节点**：`visualization (1)` $\to$ `flow_alignment (2)`。
* **禁用节点**：`io_handlers (false)`、`ground_segmentation (false)`、`clustering (false)`、`reconstruction (false)`。
* **效率监控主题**：`/flow/aligned_mesh`（统计最终渲染帧率）。

---

## 7. 启动与使用命令参考

### 7.1 常规与分布式启动
```bash
# 1. 单机全量启动（默认加载 bring_up.yaml）
ros2 launch bringup bring_up.launch.py

# 2. 分布式部署 - 开发板 1 启动：
ros2 launch bringup bring_up.launch.py board:=board1

# 3. 分布式部署 - 开发板 2 启动：
ros2 launch bringup bring_up.launch.py board:=board2

# 4. 指定外部自定义参数文件：
ros2 launch bringup bring_up.launch.py config_file:=/path/to/my_config.yaml
```

### 7.2 命令行动态日志过滤与输出控制
```bash
# 过滤掉普通 INFO，只保留 WARN 和 ERROR（排查故障极佳）：
ros2 launch bringup bring_up.launch.py log_level:=WARN

# 完全静音所有子节点终端日志（只保留 SYSTEM EFFICIENCY 监控）：
ros2 launch bringup bring_up.launch.py show_output:=false

# 组合参数：开发板 2 + 仅查看告警
ros2 launch bringup bring_up.launch.py board:=board2 log_level:=WARN
```

---

## 8. 文件结构与索引

```
src/bringup/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── bring_up.yaml          # 单机全量配置文件
│   ├── bring_up_board1.yaml   # 开发板 1 专用预设配置文件
│   └── bring_up_board2.yaml   # 开发板 2 专用预设配置文件
├── launch/
│   └── bring_up.launch.py     # 统一总起 Launch 脚本
└── src/
    └── bringup_node.cpp       # 总起节点源码（管道过滤、进程监控、效率计算）
```
