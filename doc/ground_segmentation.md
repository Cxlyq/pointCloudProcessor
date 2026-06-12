# Ground segmentation

地面分割功能包

## RANSAC

采用RANSAC拟合地面的功能包

！ RANSAC只能拟合平面地面

### 启动
```shell
colcon build --packages-select ground_segmentation
source install/setup.zsh
ros2 launch ground_segmentation gs_ransac.launch.py
```

### 频率监听
```shell
> ros2 topic hz /gs/non_ground_pointcloud

1781230944.519668 [30]       ros2: config: //CycloneDDS/Domain/General: 'NetworkInterfaceAddress': deprecated element (CYCLONEDDS_URI+0 line 1)
WARNING: topic [/gs/non_ground_pointcloud] does not appear to be published yet
average rate: 2.785
        min: 0.349s max: 0.372s std dev: 0.00897s window: 4
average rate: 2.665
        min: 0.349s max: 0.412s std dev: 0.02371s window: 7
average rate: 2.557
        min: 0.349s max: 0.446s std dev: 0.03372s window: 10

```