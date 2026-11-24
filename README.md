# Swarm 任务规划

`swarm_mission_planner` 在设备端计算能力分配、路径和多机协调。C++ 核心与 Python ROS 适配器同属这个 ROS 包。通过 rosdep 安装 Boost、OpenCV、yaml-cpp 和 nlohmann-json，构建默认使用 Release。

C++ 头文件在包根目录的 `include/swarm_planner/`，实现和计算进程入口在 `src/`。同名 Python 目录包含 ROS 适配和计算进程调用，`scripts/` 提供 ROS 节点和离线命令入口。

## 接口

- 节点 `swarm_mission_planner_node` 使用 `/swarm/planner/request` 和 `/swarm/planner/result`，消息类型为 String JSON。
- 离线命令 `ros2 run swarm_mission_planner swarm_plan INPUT.json` 读取含 `mission`、`mission_id`、`robots` 的文件并返回 JSON 计划。
- `map_package` 填写运行规划器的设备上已经部署的地图包绝对路径。
- `footprint_radius` 与 `free_thresh` 会同时传给规划和协调器预检。两端使用同一地图加载器，并按地图 YAML、图像、关系表、通道表和参数校验内容摘要。

## 算法与执行

C++ 核心执行地图解析、栅格搜索、任务插入及均衡、时间预约及 CBS 回退。输出保留原任务 ID、任务点坐标、检查点、等待点和未分配原因。Agent 从任务输入提取挂载操作。起点异常汇总设备、地图、坐标和诊断。

单图多机按规划路径的栅格占用、起终点与停车设备计算执行依赖。有依赖时须等前序路线封存并确认到达安全终点。顺序无解时明确报错。单设备支持跨图通道，多设备跨图路线暂受 ROOT 空间执行约束限制。跨图关系与通道使用 C++ 规划器的地图文件格式。

ROS 工作进程的计算期限默认 45 秒，协调器规划期限默认 60 秒。计算超时或被新请求替换时，工作进程组会被回收。结果诊断提供端到端耗时 `wall_seconds`、原生计算时间 `compute_seconds` 和原生进程峰值内存 `peak_rss_kib`。对照运行方法见 [TEST_LOG](TEST_LOG.md)。
