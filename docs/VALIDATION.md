# 验证记录与仿真验收

## 本次已执行

环境：Windows / Python 3.12；发现 WSL Ubuntu 22.04，但未安装 ROS。

- `python -B -m unittest discover -s tests -v`：17 项通过。
- `git diff --check`：通过（Windows Git 行尾转换提示不属于内容错误）。
- 本地 TARE 依赖 HEAD 与 `dependencies.repos` 固定提交一致。
- WSL 中使用 PyYAML 解析新增的 5 个 YAML/RViz/依赖/工作流配置文件：通过。
- 检查了 TARE 的真实订阅、私有参数加载、waypoint 发布、启动及返航语义。

测试中的 ROS 消息类型使用轻量替身，只验证速度保护逻辑；不证明 ROS 通信、TF 数学或规划效果正确。
尚未执行 catkin 编译、ROS 启动解析、真实 TF 变换、Gazebo 仿真和实机测试。

## Ubuntu 20.04 / Noetic 验收

先按 README 安装依赖、编译并 source 工作空间。以下是**待执行**步骤。

1. 启动解析：

   ```bash
   roslaunch --nodes alps_bringup simulation.launch
   roslaunch --nodes alps_bringup simulation.launch exploration:=false
   ```

   探索模式应出现 `sensor_coverage_planner/tare_planner_node`，手动模式应出现 `goal_bridge`。
   两种模式都只能由 `navigation_guard` 发布底盘 `/cmd_vel`。

2. 保持默认 `auto_start:=false` 启动仿真，检查数据：

   ```bash
   rostopic hz /lio_sam/mapping/odometry
   rostopic hz /lio_sam/mapping/cloud_registered_raw
   rostopic hz /state_estimation_at_scan
   rostopic hz /registered_scan
   rostopic hz /terrain_map
   rostopic hz /terrain_map_ext
   rostopic info /cmd_vel
   rosrun tf tf_echo map base_link
   rosrun tf tf_echo map sensor
   ```

   配准点云和 scan 状态时间戳应一致，frame_id 均为 map。
   不应出现重复 TF 发布、缺少 ring 字段或 IMU 初始化错误。
   未发目标/启动信号时 `/cmd_vel` 应为零，机器人不应自行驶向坐标原点。

3. 先用手动模式验证：在近处无障碍区域发送目标，再测试货架旁的目标。
   检查移动方向、地面分割、障碍物位置、转弯和目标停车。
   手动模式不是跨房间的全局路线规划验收。

4. 在新启动的探索模式中发送 `/start_exploration=true`。
   记录 `/way_point`、`/path`、轨迹及地形；检查 TARE 持续选点、局部避障。
   观察 `exploration_finish=true` 后继续返航，最终在起始位置附近停车。
   仓库参数尚未调优；若候选视点不足或探索过早结束，结合点云可见性与地形先排查数据，再调参数。

5. 中断输入验证：暂停 Gazebo，或在仿真中停止定位/规划节点。
   检查保护节点在输入超时后输出零速度。当前默认输入超时 1 秒、探索目标超时 5 秒。
   `required=true` 节点退出会触发整个 roslaunch 关闭。
   保护节点自身失效时的底盘停车属于底盘超时控制范围，不能依赖本节点自救。

6. 保存测试证据：

   ```bash
   rosbag record -O alps_acceptance.bag /clock /tf /tf_static \
     /state_estimation_at_scan /registered_scan /terrain_map /terrain_map_ext \
     /way_point /path /cmd_vel /sensor_coverage_planner/exploration_finish
   ```

   记录环境版本、TARE 提交、启动参数、碰撞次数、返航误差及是否存在长时间停滞。

## 已知边界

- Gazebo Velodyne 插件可能不提供逐点 time；LIO-SAM 在字段缺失时会关闭运动去畸变。
  应检查实际 PointCloud2 字段，不能将仿真行为直接等同于真机雷达。
- 当前桥接的状态频率等于 LIO-SAM mapping 输出频率，Python 点云变换性能尚未测量。
- 上游 LIO-SAM 的 map/odom 处理不是独立的回环校正层；探索保持回环关闭。
- 仿真传感器较低（雷达约离地 0.185 m），仓库货架遮挡与视点参数需要实测。
- 复现时请记录 ALPS-SLAM 的提交号，并通过 `dependencies.repos` 获取对应 TARE 依赖。
