# PC MSGS (Point cloud Messages Type)

该节点存储着整个工程运行时各个节点模块之间传输的数据格式

## msg/O3DPointCloud.msg

用于传递带有聚类标签的点云

使用场景： io_handler 发送原始点云给 ground_segmentation；
ground_segmentation 发送地面点云给 reconstruction。

## msg/ClusteredPointCloud.msg

用于传递带有聚类标签的点云

使用场景： clustering 聚类后发送给 reconstruction（带上 label）

## msg/O3DMesh.msg

用于传递白模重建结果

使用场景： reconstruction 发送生成的凸包/包围盒白模组合给 visualization 进行渲染

## 如何扩建自定义消息

于msg文件夹中创建<your awesome msg>.msg，打开CMakeLists.txt，向其中添加文件名
```cpp
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/O3DPointCloud.msg"
  "msg/ClusteredPointCloud.msg"
  "msg/O3DMesh.msg"
  "msg/<your awesome msg>.msg"
  DEPENDENCIES std_msgs
)
```
msg仅支持基本类型（float32, int32, string...）或ros2自带类型，因此此处为了兼任python与c++，消息采用读取成open3d数据格式后展平为一维数组向后传递，后续节点进行逆向恢复
