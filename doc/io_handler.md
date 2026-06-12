# IO Handler

处理系统IO操作的功能包

## simLidarDataFlow

模拟仿真真实雷达的数据输出流

通过读取指定文件夹下的多帧.ply文件，将其集中加载于内存后，以指定的消息频率向外发送数据

### 输出话题

输出话题名称于io_config.yaml中设定，默认/io/raw_pointcloud，消息类型 O3DPointCloud.msg

### 启动
```shell
colcon build --packages-select pc_msgs io_handlers
source install/setup.zsh
ros2 launch io_handlers io_simLidarDataFlow.launch.py
```

### 频率监听
```shell
> ros2 topic hz /io/raw_pointcloud

1781086499.521632 [30]       ros2: config: //CycloneDDS/Domain/General: 'NetworkInterfaceAddress': deprecated element (CYCLONEDDS_URI+0 line 1)
WARNING: topic [/io/raw_pointcloud] does not appear to be published yet
average rate: 7.992
        min: 0.097s max: 0.396s std dev: 0.08182s window: 12
average rate: 8.429
        min: 0.097s max: 0.396s std dev: 0.06123s window: 22
average rate: 8.497
        min: 0.097s max: 0.396s std dev: 0.05179s window: 31
```