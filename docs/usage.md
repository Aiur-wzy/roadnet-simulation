# 采集训练和查表使用说明

所有命令从仓库根目录运行。下列路径是占位示例，必须替换为已核验的真实输入和全新输出目录；源码发布不附带训练数据或已训练模型。

## 道路时间流水线

### 采集

当前原生采集以 SUMO 实际状态为准，适用于已验证的无内部道路 Euler 域。六档 label 为速度严格高于阈值的外部道路运动时间，不把完整红灯等待和路口穿越直接当作道路 label。

```bash
python -m collection.native_boundary_v2.collect_native_v2 --help
python -m collection.run_manhattan_training_batch_v2 --help
python -m data_generation.generate_manhattan_training --help
```

批采使用冻结的 dataset manifest，先 `--dry-run` 核对选择和合同，再做 `--pilot`。完整出清、关闭文件、逐ID审计和哈希核验通过后，才发布 `training.csv.gz`。`road_visits.csv.gz` 同时保留排除记录。失败场、部分文件和 `.partial` 不能作为完整训练数据。

正式批次或 `native-success-subset-v1` 成功子集均可供训练消费；成功子集是有哈希绑定的索引，不改写源批次，也不能随意移动它引用的数据。

### 模型训练

训练单位是一次车辆道路访问。正式输入为道路长度、道路/车道 flow、自由速度上限、车群类别、当前R/G及剩余秒数、下一组R/G及持续秒数；类别按无序类别处理。

```bash
python -m training.train_road_time \
  --batch-root /path/to/verified_release \
  --output-dir models/road_time_run01 \
  --sample-weighting none --memory-budget-gb 64 \
  --threshold 0p1 --time-limit 86400 \
  --bag-folds 10 --stack-levels 2 --num-gpus 0 \
  --random-state 42 --dry-run
```

64 GiB 只是示例预算。检查完整预检结果及该档 `memory_plans.*.admitted`，确认机器资源后才能去掉 `--dry-run`；干跑成功退出不代表所有拟合方案都获准。拟合仍在内存中进行，chunk只降低预检和外部评价的内存需求。

`--sample-weighting low-flow` 默认对 `road_flow < 5` 赋3倍权重后归一化，外部评价不加权；该模式检查 AutoGluon 1.6.1。train/validation按冻结family隔离，AutoGluon内部bagging不等同于family分组交叉验证。

可用阈值为 `1, 3, 0p1, 0p5, 1p5, 2`，`--all-thresholds` 按此顺序执行。`--time-limit` 约束fit，不包含完整数据预检和外部评价。每档生成predictor、模型合同及指标；全部成功才生成总训练摘要。训练输出不得覆盖旧模型，当前训练入口不支持恢复中断fit。

### 按需 catch

`catching.catch_road_time` 对场景实际访问的键迭代补齐：C++发现缺键、模型预测、再次回放。模型影响后续状态，因此后续轮仍可能出现新键。

```bash
python -m catching.catch_road_time \
  --model-dir /path/to/gt_0p1 --expected-threshold 0p1 \
  --net-file /path/to/network.net.xml \
  --output-dir catch/demand_run01 \
  --simulator build_current/Simulation_Prediction \
  --scenario example /path/to/routes.rou.xml /path/to/tripinfo.xml \
  --signal-remaining-step 1 \
  --road-flow-dense-until 10 --road-flow-step 5 \
  --first-road-position-correction off --cap-out-of-range clamp \
  --lane-discharge-interval 2 --max-rounds 30 \
  --max-cache-entries 0 --simulation-timeout 14400 \
  --min-free-disk-gib 50 --dry-run
```

路网、scope、模型和各项数值必须匹配自己的实验。示例的首路修正off用于相应大网实验；小网通常保留on，不根据文件名暗中切换。仅cap允许越界取表边界，flow、长度及未知信号组合仍受合同校验。无internal路网/无信号连接的实际穿越计时遵守C++对应规则，不用统一常数强行添加。

去掉 `--dry-run` 才会开始预测。SQLite逐批提交；中断进程确认退出后，以完全相同的输入、源码和参数加 `--resume` 恢复。不要删除检查点，或修改轮数/量化后假装仍是原合同。`--max-cache-entries 0` 是不设应用层key上限，不是无限磁盘或内存。

达到轮数上限仍缺键会保留缓存并失败退出；不等于完整表。只有零缺键并通过独立严格回放才能得到纯Table正式结果。发现轮的公式补缺结果不得混作正式成绩。

### 全量网格 catch

`catching.catch_road_time_full` 枚举声明网格内全部合法键，不运行C++需求发现，也不接收route/tripinfo：

```bash
python -m catching.catch_road_time_full \
  --model-dir /path/to/gt_0p1 --expected-threshold 0p1 \
  --net-file /path/to/network.net.xml \
  --output-dir catch/full_run01 \
  --signal-remaining-step 1 \
  --road-flow-dense-until 10 --road-flow-step 5 \
  --first-road-position-correction off --cap-out-of-range clamp \
  --chunk-size 50000 --shard-rows 1000000 \
  --min-free-disk-gib 50 --text-format none --dry-run
```

先检查精确规模估计，大网完整组合可能远超可用存储。每条二进制记录20字节，另计表头、索引和可读导出；降低分片大小不会降低总组合数。

全量恢复单位是完整分片；相同参数加 `--resume` 复用已提交片。`--text-format csv|txt|both` 增加可读副本。当前C++不能直接联合读取分片索引；可部署规模内须从首次启动指定 `--merge-output`，合成单文件 `table.demand`，并预留额外合并空间。C++仍需将该表载入内存。全量导出本身不产生交通MAE。

## 放车间隔流水线

```bash
python -m collection.collect_discharge_batch --help
python -m training.train_discharge_sequences --help
```

批采按plan和validation receipt准入，使用独立输出、资源保护和逐车终态审计。坏记录仅隔离其不可靠标签，不能跨坏车拼接间隔；整场资源中止或未完整关闭的数据不得训练。

当前训练器是标准库分层均值/中位数统计表，不是AutoGluon。首车标签 `S=t1-anchor`，后车标签 `h_j=t_j-t_(j-1)`；默认显式拟合至j=8，更长序列进入tail。条件包括放行权限、起步类型、转向和车型；稀疏支持时逐级回退，最终后车间隔回退值默认2秒。

训练前必须由外层核验finalization、audit及文件SHA，并明确域和整场holdout；单有manifest.complete不足以替代批采审计。默认整场随机划分不会自动继承旧数据的80/20列表。大小网配置域不同应分别训练：

```bash
python -m training.train_discharge_sequences \
  /path/to/accepted_train_run /path/to/accepted_validation_run \
  --holdout-run VALIDATION_RUN_ID --output models/discharge_run01 \
  --prefix-length 8 --min-samples 5 --min-sequences 3 \
  --estimator mean --fixed-headway 2
```

模型、覆盖报告和指标是独立产物。当前C++尚未消费该动态服务表；不能把首车S直接叠加到道路时间，也不能把统计表训练成功等同于闭环MAE改善。

## 复现边界

所有新任务使用独立输出，不覆盖原truth、模型或旧表。保留网络/route/SF/种子/源码指纹。历史入口供明确指定的旧合同复现，不负责接受当前native-v2数据；旧冻结文件不在源码整理时批量改名或改哈希。
