🛠️ 优化脚本使用说明
你可以通过运行 

```bash
optimize_tower_mesh.py
```

 自定义精简比例或处理其他 PLY 模型：
默认精简 50% 面数:

```bash
python3 assets/power-transmission-tower/source/optimize_tower_mesh.py
```

自定义精简比例 (例如精简 60% 面数，保留 40% 细节):

```bash
python3 assets/power-transmission-tower/source/optimize_tower_mesh.py -r 0.6
```

指定输入与输出文件:

```bash
python3 assets/power-transmission-tower/source/optimize_tower_mesh.py -i <input.ply> -o <output_optimized.ply> -r 0.5
```

仅合并重叠顶点与清理退化面 (不改变任何形变):

```bash
python3 assets/power-transmission-tower/source/optimize_tower_mesh.py --weld-only
```
