# 运行环境

## C++ 仿真

需要 CMake 3.20+ 和 C++14 编译器；当前构建不需要额外的第三方 C++ 库。Linux 可使用 GCC，Windows 可使用 MinGW 或相应的 CMake 生成器。

```bash
cmake -S . -B build_current
cmake --build build_current -j2
ctest --test-dir build_current --output-on-failure
```

已有实验的构建目录不能覆盖。Windows 下建议使用不含非 ASCII 字符的新构建路径；多配置生成器须在构建和 CTest 中选择一致的配置。

## Python

推荐 Python 3.10–3.12，在独立虚拟环境中安装所需依赖。`requirements.txt` 是依赖范围清单，不是可复现环境锁文件。

| 工作 | 依赖 |
| --- | --- |
| 数据处理、当前道路训练与 catch | NumPy、pandas |
| 真实模型拟合和推理 | AutoGluon Tabular；原模型需要匹配其训练环境 |
| 部分历史评价与绘图 | scikit-learn、matplotlib |
| 平衡需求 MILP 工具 | SciPy 1.9+ |
| 放车序列统计表训练 | Python 标准库 |

完整依赖可用 `python -m pip install -r requirements.txt` 安装。低流量加权训练另有明确的 AutoGluon 1.6.1 版本检查，不能仅凭最低版本要求认定兼容。

训练参数是 `--num-gpus`。CPU 模式可用；申请 GPU 数不得超过当前进程实际可见设备，也不保证所有候选模型使用 GPU。GPU 与 bagging/stacking 环境检查入口为：

```bash
python -m tools.environment.check_autogluon_gpu --help
```

## SUMO 和 TraCI

安装能够读取输入路网版本的 SUMO，确保 `sumo`/`netconvert` 可执行文件以及与之匹配的 TraCI 可用。随仓库附带的小路网格式为 1.20；真实实验还应冻结 SUMO 版本、随机种子、步长、车型、变道和 teleport 配置。

例如使用 SUMO 安装目录的工具包：

```bash
export SUMO_HOME=/path/to/sumo
export PATH="$SUMO_HOME/bin:$PATH"
export PYTHONPATH="$SUMO_HOME/tools${PYTHONPATH:+:$PYTHONPATH}"
sumo --version
python -c "import traci; print(traci.__file__)"
```

native-v2 道路采集当前面向无内部道路的 Euler 域；不能自动外推到 Ballistic、不同 action 步长或有内部道路的配置。

## 测试与可选依赖

单元测试的轻量环境至少安装 `numpy pandas scikit-learn`；历史训练模块的导入也需要 scikit-learn。mock 训练测试不进行真实 AutoGluon 拟合，不能据此认定模型环境可用。

```bash
python -m tools.validation.run_python_tests --list
python -m tools.validation.run_python_tests
python -m tools.validation.run_python_tests --include-sumo
```

最后一项在独立 Python 进程执行真实 SUMO 小样例，不启动全量采集。Bash 编排测试需要 Bash 与相应 Unix 工具；历史数据、真实模型或工具缺失的跳过项不能当作已验证。C++ 读表联动测试可通过 `ROADNET_TEST_BUILD_DIR` 指向新构建。
