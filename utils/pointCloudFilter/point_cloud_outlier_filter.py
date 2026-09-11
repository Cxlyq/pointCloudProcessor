import os
import glob
import open3d as o3d
import argparse

def process_outliers(input_folder, nb_neighbors, std_ratio):
    # 创建输出文件夹 OutlierFilter
    output_folder = os.path.join(input_folder, "OutlierFilter")
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # 获取所有 .ply 文件
    ply_files = glob.glob(os.path.join(input_folder, "*.ply"))
    
    if not ply_files:
        print(f"在 {input_folder} 中没有找到任何 .ply 文件。")
        return

    for file_path in ply_files:
        file_name = os.path.basename(file_path)
        
        # 截取第17位字符到末尾 (索引16)
        if len(file_name) >= 17:
            suffix = file_name[16:]
        else:
            print(f"警告: 文件名 '{file_name}' 长度不足17位，将使用原名作为后缀。")
            suffix = file_name
            
        new_file_name = f"pc_OutlierFilter-{suffix}"
        output_path = os.path.join(output_folder, new_file_name)
        
        # 读取点云
        pcd = o3d.io.read_point_cloud(file_path)
        
        if not pcd.is_empty():
            original_point_count = len(pcd.points)
            print(f"正在处理: {file_name} (包含 {original_point_count} 个点)...")
            
            # 使用统计滤波移除离群点
            # cl是清理后的点云，ind是保留点的索引
            cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
            
            # 保存处理后的点云
            o3d.io.write_point_cloud(output_path, cl)
            print(f"处理完成: {new_file_name} (剩余点数: {len(cl.points)}, 剔除点数: {original_point_count - len(cl.points)})")
            print("-" * 50)
        else:
            print(f"警告: {file_name} 是空文件或无法读取。")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="对点云进行离群点过滤 (基于统计滤波)。")
    parser.add_argument("--input", type=str, required=True, help="输入包含 .ply 文件的文件夹路径")
    parser.add_argument("--neighbors", type=int, default=50, help="统计滤波: 用于计算平均距离的邻居数量")
    parser.add_argument("--std_ratio", type=float, default=2.0, help="统计滤波: 标准差倍数阈值")
    
    args = parser.parse_args()
    
    print(f"开始离群点过滤任务...")
    print(f"输入文件夹: {args.input}")
    print(f"参数: 邻居数={args.neighbors}, 标准差倍数={args.std_ratio}")
    print("=" * 50)
    
    process_outliers(args.input, args.neighbors, args.std_ratio)
    print("所有文件处理完毕！")
