# SUMO 原生道路采集

`collect_native_v2.py` 是当前流式道路标签采集入口，`streaming_quality_v2.py` 负责逐道路质量准入。旧采集器及有界 probe/reconcile 工具保留作独立复现，不与当前正式输出混用。

## 数据合同

- feature 来自 SUMO 首次真实可见快照：实际道路/车道 flow、实际前车、速度上限和两组 R/G 信号；不替 SUMO 选择车道，不使用 C++ 影子 waiting buffer。
- 六档标签积分 `speed > {0.1, 0.5, 1, 1.5, 2, 3}` 的外部道路时间。已验证的 Euler 几何分摊用于估计跨道路的时间归属，不能消除所有离散采样误差。
- 只有可靠标签和真实有效 feature 同时具备的道路行进入 `training.csv.gz`；`road_visits.csv.gz` 保留完整访问账本与排除原因。不为整路漏快照补造 feature。
- 实际 depart/arrival 保持 SUMO 事件时钟；不是对所有 TraCI 时间统一减一个步长。末路结合完整关闭后的 tripinfo 审计。
- 只有正常结束、逐车/逐路计数闭合、输入和源码指纹稳定、输出解析及哈希通过，才发布训练数据。中断、停滞和未知状态不能充当完整 truth。

## 使用

从仓库根目录执行：

```bash
python -m collection.native_boundary_v2.collect_native_v2 --help
python -m collection.run_manhattan_training_batch_v2 --help
```

批量采集前先核对冻结 dataset manifest、输出目录及资源限制，再运行小规模 pilot。批次的停滞或失败不授权自动重试。

当前已验证采集域为 no-internal-links 的 Euler 配置。有内部道路和其他积分/动作步长的适配未完成；不能仅改变 schema 字符串让旧数据通过训练准入。probe 工具的128车限制属于有界诊断，不是流式正式采集器的规模上限。

训练和 catch 的消费标准见 [流水线使用说明](../../docs/usage.md)。
