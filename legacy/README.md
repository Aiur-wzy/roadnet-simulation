# 独立历史实现

这里的代码与当前 ABC_RG 入口物理分开，用于指定的旧特征/旧模型复现。

- `training/`：v1/v1_no_change/v2、四类车群、低流量 SF 包装、车型扩展旧入口。
- `catching/`：v1、四类车群导出与转换、车型扩展旧入口。
- `collection/`：旧车型扩展采集入口。
- `evaluation/`：旧 v1 单路拟合评价。
- `data_generation/`：原 `train_data/` 中与根生成器重复的历史副本。

使用 `python -m legacy.<功能>.<模块名> ...`，不再从根目录执行旧文件。仍被当前模型共用的旧基础模块不在此处，而在功能包 `_shared/` 中。

注意：两个旧 `model_evaluation_v1*` 没有 argparse CLI，执行 `--help` 也可能加载固定路径模型、写结果。仅在明确配置和授权下运行，不能作无副作用 smoke test。
