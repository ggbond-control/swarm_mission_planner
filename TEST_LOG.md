# Swarm 规划器测试记录

## 2026-11-16

单元测试在 `test/test_planner.py` 与 `test/test_worker.py`。覆盖地图和起点诊断、能力和操作保留、显式通道、禁止切角、阈值等值、搜索预算、跨图多机保护，以及独立进程超时回收、重复请求与后续请求。跨包计数和 ROS 集成统一记录于 [Swarm TEST_LOG](../swarm_mission_coordinator/TEST_LOG.md)。

### 本地合成基准

环境：i7-13700K、x86_64、Python 3.12。5 台设备，20 个点，0.1m 分辨率，全白地图，分离能力通道；每条任务只有一台匹配设备，路线互不冲突。每次请求重新 spawn 子进程并读取地图，不复用热缓存。以下 wall 时间包含启动和通信，峰值内存为 Linux 子进程 RSS。

| 栅格边长 | 两次 wall 时间 | 两次计算时间 | 峰值 RSS（最大值） |
| --- | --- | --- | --- |
| 128 | 0.236／0.234 秒 | 0.096／0.096 秒 | 54.4 MiB |
| 256 | 0.547／0.548 秒 | 0.406／0.406 秒 | 58.1 MiB |
| 512 | 1.845／1.855 秒 | 1.700／1.719 秒 | 70.6 MiB |

原始数据：协调器 `runtime/validation/review-20261116/benchmark.json`，日志 `20261116-review-benchmark.log`。这是便于复现的简单基准，不是原 C++ 的对比实验，也不覆盖多候选能力、迷宫、实际大图、重负载或开发板；不能据此承诺 RK3588 实时性能。

复现（已 source 新系列 install，在 src 执行；输出选独立数据目录）：

```bash
python3 swarm_mission_planner/test/benchmark_planner.py \
  --output /tmp/swarm-benchmark/result.json \
  --sizes 128 256 512 --robots 5 --tasks 20 --repeat 2 --timeout 15
```

目标板验收建议：代表性真实地图与典型／上限点数，至少统计耗时分位数、最慢值、峰值内存、失败率、路线质量及导航 CPU 负载；先补齐功能差异，再判断 Python 或独立 C++ 核心的取舍。未经这一步，不将当前新规划器标为旧版生产替代。
