# 规划器测试记录

## 2026-11-24

C++ 头文件和实现分别位于 include/swarm_planner、src。删除未实现且无调用的 PlanExporter 和 ExportOptions 声明，显式声明 Boost 编译依赖。Release 构建及八项规划器、工作进程测试通过，最终跨包结果见 [Swarm TEST_LOG](../swarm_mission_coordinator/TEST_LOG.md)。

| 文件 | 覆盖 |
| --- | --- |
| test/test_planner.py | 地图摘要和旋转、非法起点汇总、能力及操作保留、通道端点、多机跨图限制 |
| test/test_worker.py | 超时回收、失败恢复、重复请求及新请求替换 |

## 2026-11-23

512×512 合成地图，六台模拟设备，30 点和 100 点各重复 20 次。任务归属和每台顺序均与原 C++ CLI 一致。

| 点数 | 迁移版端到端 P95 | 原 CLI P95 | 峰值 RSS |
| --- | --- | --- | --- |
| 30 | 1.063 秒 | 0.991 秒 | 54.4 MiB |
| 100 | 2.949 秒 | 2.865 秒 | 55.5 MiB |

环境为本机 x86_64 Release。工作进程计时包含启动和通信，原 CLI 包含图形导出。资料在协调器 runtime/validation/planner-benchmark-20261123，开发板性能须在实际导航负载下验证。

## 复现

从工作区根目录加载编译环境后执行。

```bash
colcon test --packages-select swarm_mission_planner
python3 src/swarm_mission_planner/test/benchmark_planner.py \
  --output /tmp/swarm-benchmark/result.json --sizes 512 \
  --robots 6 --tasks 100 --repeat 20 \
  --legacy-cli build/capability_mission_planner/capability_mission_planner_cli
```

历史 Python 原型的指标保存在协调器 runtime/archives/test-log-before-review-20261124.md，对当前 C++ 实现的验收使用上述测试。
