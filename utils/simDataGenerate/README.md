# 仿真地形与无人机机载雷达扫描数据生成工具集 (simDataGenerate)

本工具集包含两个用于仿真地面三维地形白模及无人机机载雷达扫描的 Python 脚本，专为点云处理与地理配准仿真设计。

---

## 运行环境

推荐使用本工程指定的 conda 环境：
```bash
conda activate /home/asahichinn/miniconda3/envs/pointcloud_607
```

依赖核心库：
- `numpy`
- `scipy`
- `open3d` (用于快速 BVH 光线追踪 `RaycastingScene` 及网格处理)

---

## 1. 地面场景白模生成器 (`generate_terrain_meshes.py`)

### 功能介绍
- 输入所需生成的场景个数；
- 采用常见地形生成算法：
  - **分形布朗运动 (fBm / Multi-Octave Perlin Noise)**：模拟连绵起伏的山丘与平原；
  - **Diamond-Square 菱形-方形算法 (中点位移法)**：经典分形地形生成；
  - **山脊噪声 (Ridge Noise)**：模拟险峻陡峭的山脊与深谷；
  - **断层位移算法 (Fault Formation)**：模拟地质断层台地与阶梯地形；
  - **随机混合模式**：每个场景自动采用不同地貌风格与独立随机种子。
- 场景尺寸：**X 轴 6km × Y 轴 8km**；
- 网格顶点数：$51 \times 81 = 4131$ 顶点；
- 面数控制：划分网格为 $50 \times 80 \times 2 = 8000$ 个标准三角面；
- 格式：输出保存为标准 `.ply` 三维网格文件（白模赋予中性灰白材质）。

### 命令行使用方法
```bash
python utils/simDataGenerate/generate_terrain_meshes.py -n 5 -o ./terrain_models
```

### 参数说明
| 参数 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `-n`, `--count` | int | 3 | 生成的地形场景白模个数 |
| `-o`, `--output_dir` | str | `./terrain_models` | 保存生成的 `.ply` 白模文件夹路径 |
| `--algorithm` | str | `random` | 地形风格 (`random`, `rolling_hills`, `mountainous`, `ridge_valley`, `diamond_square`, `fault_plateau`) |
| `--width` | float | 6000.0 | X 轴范围（米） |
| `--height` | float | 8000.0 | Y 轴范围（米） |
| `--seed` | int | None | 随机种子 |
| `--ascii` | flag | False | 保存为 ASCII PLY 格式（默认二进制 Binary PLY，更快更紧凑） |

---

## 2. 无人机飞行与机载雷达扫描仿真器 (`simulate_radar_scanning.py`)

### 功能与物理建模
1. **坐标系与航线规划**：
   - 场景局部直角坐标系以 6km 边长为 X 轴正半轴，8km 边长为 Y 轴正半轴；
   - 无人机起点位于 $(3\text{km}, 0\text{m})$ 处，该点定义为 **GPS 坐标 (0, 0) 基准点**；
   - 无人机在离地约 300~400 米高度上（默认 350m），笔直平行于 Y 轴向前方飞行；
   - 巡航航速：$30\text{ m/s}$，航向角 (Course) 为 $0^\circ$（正北）。
2. **机载雷达扫描与动态步长**：
   - 扫描时序：每 2 秒扫描一帧，连续扫描 20 帧（无人机前飞 $0 \sim 1140\text{ m}$）；
   - 水平视场角 (Azimuth FOV)：$120^\circ$（航向左右各 $\pm 60^\circ$）；
   - 俯仰扫描角 (Pitch)：从正下方 ($-90^\circ$) 到正前方扇区；
   - **动态步长算法**：
     - 若采用等角俯仰扫描，地面距离 $y = H / \tan(\psi)$ 随俯仰角升高呈平方量级发散（$\Delta y \propto 1 / \sin^2\psi$），远距离点云将极度稀疏；
     - 本算法在地面探测距离空间内进行均匀/自适应受控步长划分，动态反求每条光线的俯仰角 $\psi = \arctan(H / y)$，在远距离自动加密角步长，确保地面点云在几千米外依然连续且均匀；
   - 每帧总点数：约 15,000 点（150 条水平线 × 100 步俯仰动态光线）；
   - 基于 Open3D `RaycastingScene` BVH 进行毫秒级精确空间网格求交。
3. **高压线与高压铁塔规划**：
   - 在 $6\text{km} \times 8\text{km}$ 场景中模拟多条合法直线走廊作为高压输电线；
   - 铁塔按标准电力工程档距（默认 350m）等间距布设；
   - 铁塔海拔由对应地面真实网格高程推算；
   - 铁塔 GPS 坐标以 $(3\text{km}, 0)$ 为 (0, 0) 基准结合 WGS-84 椭球精准推算。

### 输出文件格式
对每个地面白模场景，生成独立子文件夹，包含以下三项：
1. `pointclouds/`：存放 20 帧扫描后的 `.ply` 地面点云（包含 `x, y, z, power` 属性，默认以无人机局部坐标系保存，雷达所在局部坐标严格为 `(0, 0, [飞行高度])`，即航向为 Y，横向为 X，高程为 Z）；
2. `extend_data.json`：对齐 `dataset/Wild/04/extend_data.json`，记录每帧对应的 `ply_filename` 及无人机在扫描时的 GPS 经纬高与航向角；
3. `powerLineGPS.json`：对齐 `dataset/Wild/04/powerLineGPS.json`，记录 `HVL_1`, `HVL_2`, `HVL_3` 等高压线铁塔的 GPS 与高程信息。

### 命令行使用方法
```bash
python utils/simDataGenerate/simulate_radar_scanning.py \
    -i ./terrain_models \
    -o ./sim_radar_output \
    --speed 30.0 \
    --height 350.0 \
    --frames 20 \
    --interval 2.0 \
    --points 15000 \
    --fov 120.0 \
    --radar_z_mode agl
```

### 参数说明
| 参数 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `-i`, `--input_dir` | str | `./terrain_models` | 存放地面白模 `.ply` 文件的目录 |
| `-o`, `--output_dir` | str | `./sim_radar_output` | 仿真数据输出根目录 |
| `--speed` | float | 30.0 | 无人机飞行速度 (m/s) |
| `--height` | float | 350.0 | 无人机离地巡航高度 (m, 范围 300~400m) |
| `--frames` | int | 20 | 扫描帧数 |
| `--interval` | float | 2.0 | 每帧扫描时间间隔 (秒) |
| `--points` | int | 15000 | 每帧目标雷达点云点数 |
| `--fov` | float | 120.0 | 雷达水平扫描角 (度) |
| `--base_lat` | float | 0.0 | 场景 (3km, 0) 对应的 GPS 纬度基准 (默认 0.0) |
| `--base_lon` | float | 0.0 | 场景 (3km, 0) 对应的 GPS 经度基准 (默认 0.0) |
| `--coord_mode` | str | `local` | 点云坐标系: `local` (机体局部坐标) 或 `scene` (场景世界坐标) |
| `--radar_z_mode` | str | `agl` | 雷达局部坐标Z轴基准: `agl` (相对地面高度，如350m，雷达位于 `(0,0,height)`) 或 `abs` (绝对海拔高度，雷达位于 `(0,0,Altitude)`) |
| `--power_lines` | int | 3 | 生成的合法高压线数量 |
| `--tower_spacing` | float | 350.0 | 高压铁塔档距（米） |
| `--binary` | flag | False | 保存为二进制 PLY（默认保存为 ASCII PLY） |
