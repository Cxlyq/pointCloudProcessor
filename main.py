# main.py
import os
import glob

# 注意：必须提前导入所有的 modules，这样装饰器 @AlgorithmFactory.register 才会生效，把算法注册进工厂
import modules.io_handlers.pcd_loader
import modules.io_handlers.data_saver
import modules.ground_segmentation.ransac_segmenter
import modules.clustering.polar_pdbscan
import modules.reconstruction.convex_hull
import modules.visualization.streaming_viewer

from core.pipeline import PipelineController


def main():
    # 1. 初始化总控（此时会读取 YAML，加载模块，并直接弹出空白的 3D 渲染窗口等待数据）
    config_path = "configs/default_pipeline.yaml"
    controller = PipelineController(config_path)

    # 2. 准备数据集路径
    dataset_dir = os.path.join("dataset", "pointCloud", "data1", "City", "01", "dense")
    search_pattern = os.path.join(dataset_dir, "*.ply")
    ply_files = sorted(glob.glob(search_pattern))

    if not ply_files:
        print(f"[ERROR] 错误: 未找到任何 .ply 文件。路径：{search_pattern}")
        return

    print(f"\n[INFO] 找到 {len(ply_files)} 帧点云，开始流式处理...")

    # 3. 开始流式处理循环
    for file_path in ply_files:
        frame_id = os.path.splitext(os.path.basename(file_path))[0]

        print(f"[INFO] 处理帧: {frame_id} ...", end="", flush=True)

        # 每一帧进入流水线，处理完毕后，StreamingViewer 会立刻在挂起的窗口中刷新这帧的白模
        metrics = controller.process_frame(frame_id, file_path)

        # 你可以根据 metrics 打印这帧的耗时
        total_time = metrics.get('loader_time', 0) + metrics.get('ransac_time', 0) + \
                     metrics.get('clustering_time', 0) + metrics.get('mesh_time', 0) + \
                     metrics.get('render_time', 0)

        print(f"[INFO] 完成! 耗时: {total_time:.4f}s | 白模数量: {metrics.get('valid_hull_count', 0)}")

    print("\n[INFO] 所有数据帧处理完毕！")

    # 4. 阻止程序退出，保持 3D 窗口开启，直到用户手动点击 X 关闭
    controller.shutdown()


if __name__ == "__main__":
    main()