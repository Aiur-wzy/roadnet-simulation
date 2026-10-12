# RoadNet / CAMS

面向 SUMO 的粗粒度、事件驱动路网仿真系统。C++ 用道路行驶时间、信号窗口、车道容量和放行队列推进车辆；Python 完成需求生成、TraCI 采集、模型训练、查表导出和评价。

本仓库公开源码、测试、使用说明及必要的小样例。完整训练数据、模型、生成表、实验报告、论文和本地项目手册不随源码发布；已有本地文件不会因 `.gitignore` 被删除。

## 功能目录

```text
simulator/          C++ 建图、事件调度、车道和 movement、公式及 Table
collection/         SUMO 原生道路标签与放车序列采集
training/           道路时间和放车序列训练的 Python 核心入口
catching/           按需 catch、全量网格 catch 的 Python 核心入口
features/           特征和序列合同
data_generation/    路线库、需求、显式 speedFactor 和信号生成
evaluation/         仿真运行、评分和诊断工具
tools/              环境检查、验证和通用维护工具
scripts/            按 collection/training/evaluation 分类的 shell 编排
legacy/             独立历史算法入口
tests/              Python/C++ 测试和小型 fixtures
docs/               公开的环境、目录和流水线使用说明
data/net/           小型路网 fixture
catch/              本地生成表的存放位置，不是导出代码
```

`training/`、`catching/` 不与 `.sh` 脚本混放。旧 SF/ABC/ABC_RG 入口及 `_shared/` 仍保留，供明确指定的历史复现使用；当前 native-v2 训练和 catch 不导入旧模型流水线。旧文件名到现位置的映射见 [source_layout.json](docs/source_layout.json)。

## 构建与快速检查

需要 CMake 3.20+ 和 C++14 编译器。Python 依赖及 SUMO 设置见 [环境说明](docs/environment.md)。从项目根目录运行：

```bash
cmake -S . -B build_current
cmake --build build_current -j2
ctest --test-dir build_current --output-on-failure
./build_current/Simulation_Prediction --use-sumo \
  --sumo-net data/net/test_new.net.xml --smoke-test
python -m tools.validation.run_python_tests
```

Windows 使用相应 `.exe`；多配置生成器还需选择一致的配置。构建应使用新目录，不能覆盖运行中或暂停实验所依赖的二进制。

Python 默认测试不启动正式训练或全量采集；部分可选测试依赖 Bash、SUMO 或本地历史数据，会按环境跳过。`--include-sumo` 在独立 Python 进程补跑小型 SUMO 集成测试，避免 TraCI mock 污染真实接口。

## 当前入口

所有 Python 入口从仓库根目录以 `python -m` 调用；Linux 可使用 `python3`。

| 工作 | 模块 |
| --- | --- |
| 道路时间采集 | `collection.native_boundary_v2.collect_native_v2` |
| 道路时间批采 | `collection.run_manhattan_training_batch_v2` |
| 训练需求生成 | `data_generation.generate_manhattan_training` |
| 道路时间训练 | `training.train_road_time` |
| 按需 catch | `catching.catch_road_time` |
| 全量网格 catch | `catching.catch_road_time_full` |
| 放车序列批采 | `collection.collect_discharge_batch` |
| 放车序列训练 | `training.train_discharge_sequences` |

例如 `python -m training.train_road_time --help`。输入合同、预检、恢复和部署边界见 [流水线使用说明](docs/usage.md)。

## 数据与版本边界

- 道路采集以 SUMO 实际快照为准，不构造 C++ 影子 flow 或替 SUMO 选道。native-v2 的已验证采集域为无内部道路的 Euler 配置；不能把旧 CSV 改名当作新数据。
- 训练只消费经过终态、质量和哈希核验的正式批次或成功子集；道路时间模型与放车序列模型是不同任务。
- 按需 catch 只有零缺键后的严格重放才能作为纯 Table 结果。全量网格导出则需先检查组合规模。
- 放车序列训练目前生成独立统计表，尚未自动接入 C++ 动态放行。首车起步时间不能直接重复加入道路行驶时间。
- 本地研究工作区可保留 `PROJECT_MANUAL.md` 作为唯一状态和 TODO 手册；公开检出的代码不依赖该文件运行。

发布范围及历史复现规则见 [目录与发布说明](docs/project_layout.md)。源码同步不改写已有实验 manifest、模型合同或旧文件哈希。
