# 数据目录

公开仓库只保留 `net/test_new.net.xml` 作为静态建图与测试 fixture：

```bash
./build_current/Simulation_Prediction --use-sumo \
  --sumo-net data/net/test_new.net.xml --smoke-test
```

`route/`、`sumocfg/`、`tripinfo/`、`variants/` 等属于本地实验输入输出，不随当前源码发布。原 free/tight/congested 三组数据及其 truth 原位保留，不因取消 Git 跟踪而删除；旧提交中的公开副本仍可追溯。

SUMO 重跑时请显式指定新的输出目录，避免覆盖已有 tripinfo。网络 fixture 不代表当前 native-v2 采集器已支持其中的内部道路。
