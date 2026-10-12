# 功能目录与发布范围

核心代码按功能组织，当前入口与独立历史入口分开；文件的科学有效性不以日期或文件名判断。

## 当前与历史代码

道路时间当前流水线是 `collection/native_boundary_v2/`、`training/train_road_time.py` 和 `catching/catch_road_time.py`；全量网格导出使用 `catching/catch_road_time_full.py`。放车序列代码也按采集、特征、训练分别归属相应包。

`legacy/` 存放独立旧入口。`collection/_shared/` 仍是采集依赖；`training/_shared/` 和 `catching/_shared/` 服务历史 SF/ABC/ABC_RG 入口，不是当前 native 模型的兼容层。不要仅因名称较旧就删除它们。

`scripts/` 只放 shell 编排；环境、校验和维护 Python 工具放在 `tools/`。目录迁移不改变特征、标签、数据内容或仿真算法。

## 旧入口迁移

旧路径与新位置的完整映射在 [source_layout.json](source_layout.json)，运行时定位由根目录 `roadnet_paths.py` 提供。Python 模块从仓库根目录运行，例如：

```bash
python -m training.train_road_time --help
python -m catching.catch_road_time --help
python -m catching.catch_road_time_full --help
```

同步到另一台机器时应同步完整相关包和 `roadnet_paths.py`，不能只复制一个顶层入口。历史冻结任务继续使用其原源码快照；不批量修改旧 manifest 的路径或哈希。

## 公开仓库和本地研究工作区

| 类型 | Git 发布规则 | 本地处理 |
| --- | --- | --- |
| 源码、测试、必要 fixture | 发布 | 按功能目录维护 |
| 使用说明 | 仅发布 `.gitignore` 中列出的文档 | 其他报告原位保留 |
| 训练数据、模型、TraCI、生成表 | 不发布 | 保留原路径、合同和来源 |
| 论文、完整研究手册、实验报告 | 不发布 | 仍在本地维护 |
| 构建、缓存、日志、冻结操作脚本 | 不发布 | 不影响运行或暂停任务依赖 |

历史提交中已公开的实验产物仍可从 Git 历史检索；从当前版本取消跟踪不是历史擦除，也不是删除本地文件。不要使用 `git clean -fdx` 清理研究工作区。

发布前应检查暂存文件清单、大小和凭据，并在不含被忽略数据的新检出目录编译及测试。任何数据覆盖、训练、采集和服务恢复都不是同步源码的隐含步骤。
