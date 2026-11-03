# 地面校正算法验证数据集生成与雷达仿真工具

本文件夹 (`utils/verify/ground_correction_verify`) 包含针对地面校正与机载雷达地面滤波算法的两个核心验证仿真脚本：

1. **`generate_ground_meshes.py`**：地面白模批量生成器。根据指定个数，利用常见地形生成算法，批量生成 6km × 4km、约 4000 面的 `.ply` 地面几何白模。
2. **`simulate_radar_scan.py`**：机载雷达对地扫描模拟器。输入 `.ply` 地面白模文件夹，从地面中轴线一端、离地 300~400 米高度对地面进行水平 120°、俯仰从正下方 (-90°) 到正前方/正上方 (+90°) 的雷达扫描，输出约 15000 点的 `.ply` 地面点云。

---

## 运行环境

推荐使用配置好的 Conda 环境：
```bash
conda activate pointcloud_607
# 或直接调用该环境解释器：
# /home/asahichinn/miniconda3/envs/pointcloud_607/bin/python
```
依赖库：
- `numpy` (>= 1.20)
- `scipy` (>= 1.7)
- `open3d` (>= 0.15)

---

## 1. 地面白模批量生成脚本 (`generate_ground_meshes.py`)

### 1.1 核心算法
本脚本内置了 6 种经典且具有不同几何地貌特征的地形生成算法，生成过程会自动循环或挑选不同算法，并使用不同随机种子确保地貌的多样性：
1. **分形布朗运动丘陵 (Perlin / fBm Multi-octave Gradient Noise)**：多频自相似噪声，模拟典型连绵自然丘陵与缓坡谷地。
2. **菱形-方形分形山脉 (Diamond-Square Algorithm)**：经典图形学等离子分形高度场，模拟陡峭险峻的山峰、深壑与山脊。
3. **多高斯起伏原野 (Multi-Gaussian Rolling Hills & Basins)**：随机放置正负高斯核，模拟开阔平缓的丘陵原野与盆地洼地。
4. **断层阶梯台地 (Stepped Fault Plateaus & Terraces)**：多条随机断层位移线叠加阶梯化算子，模拟断裂带、大峡谷与梯状高原。
5. **尖锐山脊峡谷 (Ridged Multifractal Noise)**：反转绝对值算子 $1 - |\text{noise}|$，构建刃状山脊线与狭长深谷。
6. **风蚀波纹沙丘 (Wind-blown Sand Dunes)**：非对称波浪与风蚀扰动，模拟沙漠沙丘与平行波纹地形。

### 1.2 空间范围与网格规格
- **几何范围**：长 $L_x = 6000.0\text{ m}$ (6 km), 宽 $L_y = 4000.0\text{ m}$ (4 km)。
- **坐标系设置**：X 轴范围 $[0, 6000\text{ m}]$，Y 轴范围 $[-2000\text{ m}, +2000\text{ m}]$（中心轴线位于 $Y=0$），Z 轴为地表高程。
- **网格面数**：网格分辨率设为 $57 \times 38 = 2166$ 顶点，由 $56 \times 37$ 个近正方形单元格剖分为 **4144 个三角面**（非常接近 4000 面）。
- **白模属性**：设置统一的白色/米白顶点色彩 $(245, 245, 245)$，并自动计算法向量，导出标准的二进制 `.ply` 网格。

### 1.3 命令行参数与运行示例
```bash
# 基本用法：生成 5 个白模到默认目录 output_ground_meshes
python generate_ground_meshes.py -n 5

# 指定输出目录与目标面数
python generate_ground_meshes.py -n 10 -o ./my_ground_meshes --target_faces 4000

# 交互式运行（不加参数时会提示输入个数）
python generate_ground_meshes.py
```

---

## 2. 机载雷达地面扫描模拟脚本 (`simulate_radar_scan.py`)

### 2.1 扫描几何与物理模型
- **雷达站位**：
  - 自动识别白模几何包围盒的长边方向（默认 X 轴为 6km 长边，Y 轴为 4km 短边）。
  - **放置在矩形长边中心点上**：例如 $X_{radar} = (X_{min} + X_{max}) / 2 = 3000\text{m}$, $Y_{radar} = Y_{min} = -2000\text{m}$。
  - 获取该点处的地表高程 $Z_{ground}$，雷达架设在离地 $Z_{radar} = Z_{ground} + H$ 处。
  - 离地高度 $H$ 位于 300~400 米（默认 350 米，亦支持 `--random_height` 为每个地形随机选择）。
  - **前向扫描朝向**：朝向矩形内部（横贯 4km 深度方向）。
- **扫描视场与角度**：
  - **水平扫描角 (Azimuth FOV)**：$120^\circ$（以正前方为中心，方位角覆盖 $[-60^\circ, +60^\circ]$，在 4km 纵深处横向覆盖宽度达 6.9km，完整覆盖 6km 长边）。
  - **俯仰扫描角 (Pitch Range)**：从正下方 ($-90^\circ$, Nadir) 扫描至正前方/正上方 ($+90^\circ$)。
- **【核心升级】动态步长俯仰角扫描 (Dynamic Pitch Step)**：
  - **物理机理**：在机载雷达等角扫描时，地表点距公式为 $\Delta x \approx \frac{H}{\sin^2\phi}\Delta\phi$。等角步长会导致正下方点云密度是 6km 远处的数百倍。
  - **动态步长策略**：建立地表投影纵深连续非线性映射：
    - **近处（正下方 $-90^\circ$ 附近）**：俯仰角步长自适应增大至约 $6^\circ \sim 7^\circ$，有效防止近处点云过度密集成堆；
    - **远处（切向浅角 $-5^\circ$ 附近）**：俯仰角步长自适应缩小至 $0.04^\circ \sim 0.05^\circ$（密度提升 130 倍以上），确保 3~6km 远距离微小地形起伏特征具有充足细节；
    - 纵深点数分布均衡优化：近距离 (<1km) 约占 28%，中距离 (1~3km) 约占 41%，远距离 (>3km) 约占 31%。
- **光线求交与点云提取**：
  - 采用 Open3D `RaycastingScene` 进行高速 BVH 光线-三角网格求交。
  - 自动标定光线角分辨率，使最终**有效击中地面生成的点云点数约为 15,000 个**。
  - 附带高程伪彩色与模拟反射强度，输出标准 `.ply` 激光/雷达点云。

### 2.2 命令行参数与运行示例
```bash
# 默认使用【动态步长】对 output_ground_meshes 中的白模进行雷达扫描
python simulate_radar_scan.py -i ./output_ground_meshes -o ./output_scanned_pointclouds

# 自定义动态步长幂律指数（默认 1.0 为均匀地表间隔，增大数值更偏向远处）
python simulate_radar_scan.py -i ./output_ground_meshes -o ./output_scanned_pointclouds --pitch_mode dynamic --pitch_power 1.0

# 若需对比传统均匀等角度扫描效果，可切换为 uniform 模式
python simulate_radar_scan.py -i ./output_ground_meshes -o ./output_scanned_pointclouds --pitch_mode uniform

# 自定义雷达高度（例如 380 米）或开启 300~400 米随机高度
python simulate_radar_scan.py -i ./output_ground_meshes -o ./output_scanned_pointclouds --random_height

# 指定点数（默认 15000）与微小测距噪声
python simulate_radar_scan.py -i ./output_ground_meshes -o ./output_scanned_pointclouds --target_points 15000 --noise 0.05
```

---

## 3. 地面重建精度评估脚本 (`evaluate_ground_reconstruction.py`)

### 3.1 评估原理与核心机制
- **扇面区域过滤 (Sector Mask)**：
  - 以机载雷达安装的长边中心点为基准站位，严格提取水平 $120^\circ$ 扇面覆盖范围；
  - **仅对比落在点云实际覆盖扇面内的重建效果**，避免未扫描的盲区干扰评价结果。
- **高精度垂直投影真值查询**：
  - 对扇面内重建模型的每个评估点 $(X, Y)$，使用 Open3D `RaycastingScene` 向下垂直投射光线，精确获取原始地面基准高程 $Z_{truth}$；
  - 垂直误差计算：$e = Z_{recon} - Z_{truth}$。
- **核心评价指标**：
  1. **最大误差点**：偏高最严重误差点（Max Positive Error）及最大绝对误差点（Worst-case Point）的三维坐标与高程差；
  2. **最小误差点**：偏低最严重误差点（Min Negative Error）及最小绝对误差点（最吻合点）的三维坐标与高程差；
  3. **平均误差统计**：平均系统偏差（Bias）、平均绝对误差（MAE）、均方根误差（RMSE）与标准差（Std）；
  4. **85% 精准度误差范围 (P85)**：计算绝对误差的第 85 百分位数，给出“在 $\pm \epsilon_{85}$ 米误差范围内，地面重建精度达到 85%”的定量结论（同时输出 P50、P90、P95、P99 与阶梯累计分布）；
  5. **误差彩色模型导出**：
     - **红色**：代表偏高（$Z_{recon} > Z_{truth}$），误差越大红色越深；
     - **蓝色**：代表偏低（$Z_{recon} < Z_{truth}$），误差越大蓝色越深；
     - **白色**：代表无误差或误差极小（$Z_{recon} \approx Z_{truth}$）；
     - 扇面外背景点：浅灰色；
     - 支持 `--crop_sector` 额外保存纯扇面内的裁剪 `.ply` 网格/点云。

### 3.2 命令行参数与运行示例
```bash
# 基本用法：输入原始地面与重建地面，自动定位长边中心扇面并评估
python evaluate_ground_reconstruction.py \
    -g ./output_ground_meshes/ground_terrain_001_perlin_fbm.ply \
    -r ./reconstructed_mesh.ply \
    -o ./reconstructed_error_colored.ply

# 传入雷达扫描点云作为辅助参考，并导出纯扇面裁剪模型与 JSON 指标
python evaluate_ground_reconstruction.py \
    -g ./output_ground_meshes/ground_terrain_001_perlin_fbm.ply \
    -r ./reconstructed_mesh.ply \
    -p ./output_scanned_pointclouds/scanned_pcd_ground_terrain_001_perlin_fbm.ply \
    -o ./reconstructed_error_colored.ply \
    --crop_sector \
    --save_json ./eval_report.json
```

