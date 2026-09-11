import os
import glob
import numpy as np
import open3d as o3d
import argparse

def process_point_clouds(input_folder, R):
    # 创建输出文件夹 RadialShear
    output_folder = os.path.join(input_folder, "RadialShear")
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # 获取所有 .ply 文件
    ply_files = glob.glob(os.path.join(input_folder, "*.ply"))
    
    if not ply_files:
        print(f"在 {input_folder} 中没有找到任何 .ply 文件。")
        return

    for file_path in ply_files:
        file_name = os.path.basename(file_path)
        
        # 获取第17位字符到末尾的部分 (Python 索引从 0 开始，所以第17位是索引 16)
        # 考虑到有些文件名可能不足17位，做个长度判断
        if len(file_name) >= 17:
            suffix = file_name[16:]
        else:
            print(f"警告: 文件名 '{file_name}' 长度不足17位，将使用原名作为后缀。")
            suffix = file_name
            
        new_file_name = f"pc_RadialShear-{suffix}"
        output_path = os.path.join(output_folder, new_file_name)
        
        # 读取点云
        pcd = o3d.io.read_point_cloud(file_path)
        
        if not pcd.is_empty():
            points = np.asarray(pcd.points)
            
            # 计算所有点到原点 (0,0,0) 的距离
            distances = np.linalg.norm(points, axis=1)
            
            # 过滤出距离大于 R 的点 (删除半径 R 内的点)
            mask = distances > R
            
            # 创建新的点云并赋值过滤后的点
            filtered_pcd = o3d.geometry.PointCloud()
            filtered_pcd.points = o3d.utility.Vector3dVector(points[mask])
            
            # 如果原点云包含颜色，也一并过滤保留
            if pcd.has_colors():
                colors = np.asarray(pcd.colors)
                filtered_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
                
            # 如果原点云包含法向量，也一并过滤保留
            if pcd.has_normals():
                normals = np.asarray(pcd.normals)
                filtered_pcd.normals = o3d.utility.Vector3dVector(normals[mask])
                
            # 保存处理后的点云
            o3d.io.write_point_cloud(output_path, filtered_pcd)
            print(f"处理完成: {file_name} -> {new_file_name} (原点数: {len(points)}, 剩余点数: {np.sum(mask)})")
        else:
            print(f"警告: {file_name} 是空文件或无法读取。")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="删除以 (0,0,0) 为球心，R 为半径内的所有点云点。")
    parser.add_argument("--input", type=str, required=True, help="输入包含 .ply 文件的文件夹路径")
    parser.add_argument("--R", type=float, required=True, help="球体半径 R (单位：米)")
    
    args = parser.parse_args()
    
    print(f"开始处理，输入文件夹: {args.input}, 删除半径 R: {args.R}米")
    process_point_clouds(args.input, args.R)
    print("所有文件处理完毕！")
