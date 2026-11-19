# IACO-DWA Path Planning

## Overview

单台两轮差速物流机器人的 MATLAB 研究代码，依据新版 *An Optimized ACO-DWA Hybrid Path Planning Method for Two-Wheel Differential Mobile Logistics Robots* 实现 IACO 全局规划、轨迹平滑、DWA 动态避障、视线重连和分级故障恢复。实验可导出图 6–8 对应的图形、表 3 与表 4 对应的逐次日志和统计汇总。

## Requirements

完整实验脚本面向 MATLAB R2024b，使用基础 MATLAB。栅格搜索、轮盘赌、B 样条和 DWA 在本仓库中实现。运行实验不需要 Robotics System Toolbox、Spline Toolbox 或 Statistics and Machine Learning Toolbox。数值核心及 smoke_test.m 同时在 GNU Octave 9.4.0 上实测。绘图及 table/CSV 驱动使用 MATLAB 接口。运行前需对仓库目录具有写权限。

## How to Run

解压后，将 MATLAB 当前文件夹设为包含 setup_project.m 的仓库根目录。首先运行以下命令，检查地图、规划的确定性、折线碰撞检测和单车动态导航。

```matlab
run('experiments/smoke_test.m')
```

| 实验 | 在 MATLAB 命令窗口执行 | 输出 |
|---|---|---|
| 图 6，三张地图上的路径对比 | `run('experiments/run_fig6_comparison.m')` | ACO、A*、Ref-ACO 图示路径、IACO；轨迹坐标 CSV |
| 图 7，收敛曲线 | `run('experiments/run_fig7_convergence.m')` | 每一代成功蚂蚁的最短栅格路径长度，缺少成功蚂蚁的代次留空；另绘论文的 Ref-ACO 曲线 |
| 图 8，动态避障 | `run('experiments/run_dynamic_sim.m')` | 四个仿真时刻、逐帧位姿和速度、障碍物位置、避障状态、恢复阶段、净间距 |
| 表 3，全局规划统计 | `run('experiments/run_table3_comparison.m')` | 三图 × 50 个种子，计算 ACO、A*、IACO。原文 Ref-ACO 汇总见 reference/paper_table3.csv |
| 表 4，九种消融设置 | `run('experiments/run_table4_ablation.m')` | 同一动态地图，完整方法及八种单模块关闭，每组 50 次 |
| 原目录建议中的旧脚本名 | `run('experiments/run_table3_ablation.m')` | 调用新版表 4 消融实验 |

输出路径为 results/figures（PNG、PDF）及 results/logs（CSV 和 MAT 配置快照）。运行同一入口会更新其同名输出。表 4 每完成一个种子的九组试验即写入逐次日志。终端计时为运行机器上的实际耗时，可随机器负载变化。

## Parameter Tuning

| 文件 | 内容 |
|---|---|
| configs/params_aco.m | 论文表 2 的蚂蚁数 50，α=1，β=5，Q=100，先验增益 2.5、空间衰减 0.5、蒸发率上下界 0.5 和 0.1；另外列出实现采用的固定蒸发率、奖惩强度、目标势场参数等 |
| configs/params_dwa.m | 表 2 的线/角速度上限 0.5 m/s、1.2 rad/s；加速度上限 0.2 m/s²、0.5 rad/s²；四项评价权重、采样数、预警距离、TTC、得分阈值、连续净空帧数、偏差阈值及恢复持续时间 |
| configs/params_sim.m | 地图选择、起终点、初始航向、种子、试验次数、移动物体的路线与速度及超时 |
| configs/ablation_config.m | 默认全部开启。use_potential_field 同时控制势场与转弯代价。use_adaptive_evaporation 同时控制自适应挥发与精英奖惩。use_reconnection 和 use_failure_recovery 相互独立。其余开关分别控制 A* 先验、剪枝、平滑和动态风险项 |

直接修改某个开关并运行图 8，可观察单组消融。批量表 4 以当前 flags 为基准，每次再关闭一个指定开关；对照九组标准消融时请保持配置文件中八项均为 true。默认种子序列为 42 至 91，每个方法和配置在每次试验开始时分别调用 rng(seed,'twister')。相同软件版本、地图、算法和参数下可复核随机轨迹。MATLAB 和 Octave 的 RNG 实现以及处理器浮点行为可能不同。

## 地图、原文数值与参数来源

- maps 下的四个 MAT 文件均含 occupancy（二值矩阵，1 为障碍）和 cell_m。MATLAB 的第 1 行对应最低的 y，第 1 列对应最小的 x。格心坐标为 ([列,行]−0.5)×cell_m。静态场景的栅格边长按图示刻度取 1 m，每张地图都从文件加载。
- map_20x20.mat、map_25x25.mat、map_30x30.mat 由新版图 6 的矢量障碍方块重建，分别包含 135、164 和 243 个占据格。map_dynamic_test.mat 由图 8(a) 重建。图 8 坐标轴显示 1–30，扩展至 0 的边缘区域按可辨认矢量块填充。图中的黄、绿圆单独建模，绿圆的折线路径、速度与半径由配置定义。
- 原始仿真 MAT 文件、初始种子列表、完整的局部控制器权重、各次试验的传感器记录及 Ref-ACO 的源码均未随新版稿件提供。这里使用图示地图，按公开公式编写算法；未列出的阈值、离散化与运动参数采用公开在 configs 中的工程设定。图 8 模拟器用范围内障碍物的理想位置和速度模拟跟踪器输出。
- reference/paper_table1.csv 至 paper_table6.csv 是新版各表的直接转录。表 3 原表列出 ACO、Ref-ACO、IACO；本仓库额外计算 A*，与新版图 6 对应。ref_path_*.csv 与 ref_curve_*.csv 是从新版图 6、7 蓝色矢量曲线按坐标刻度反算的数据。图例明确标为 digitized。源码计算不会读取论文表中数值作为实验输出。表 5、6 的报告值仅供查阅，当前实验入口覆盖图 6–8、表 3–4。
- 单次研究复跑可能产生与文中不同的结果。新版表 4 完整方法报告的路径长度为 43.26 m，最小安全距离为 0.82 m；verification 下保留了本实现一次独立检查的日志摘要，可据此区分数据来源。

## 算法模块与指标

src/global 生成八邻域图，禁止斜穿障碍角。A* 计算先验通路，IACO 采用按距先验折线衰减的信息素、含目标吸引和转角代价的启发函数、按迭代变化的挥发率和同代奖惩。经典 ACO 通过共享的轮盘赌/信息素迭代器使用均匀先验、距离倒数和固定挥发率。没有成功蚂蚁时会保留规划失败记录。src/smoothing 完成视线剪枝和夹持三次 B 样条，输出采样折线经足迹碰撞检查，若有障碍则回退至平滑前的可行路径。

src/local 在速度与加速度窗口内预测差速轨迹，筛选静态净空、动态间距与制动距离。DWA 对航向、净空、速度和相对接近风险加权。控制循环检查预警半径、预测碰撞时间和得分阈值，依据连续净空帧、可视航点和偏差阈值重连。异常时依次尝试制动重采样、原地旋转、短距离倒车、临时目标和局部 IACO 重规划。恢复动作也经过候选轨迹净空检查。

表 3 统计最终航点路径的长度、首次达到全程最优栅格代价的代数、按阈值统计的方向改变数、算法运行时间和成功百分比。转向数在平滑前保留的控制折线上统计，A* 的迭代代数留空。表 4 的长度是成功试验实际行驶里程。最小距离为车体边缘至障碍或场地边界的逐帧最小净距；碰撞率分母为全部试验。角速度 RMS 按控制周期计算。重连时间从连续净空开始计至恢复距离和航向容差，按成功且存在完整重连事件的试验取均值，同时报告有效样本数。未完成的试验仍保留成功、规划和碰撞标记。

## 验证记录与许可

GNU Octave 9.4.0，种子 42。已运行三张静态地图上的 A*、经典 ACO 和 IACO，以及四张地图读取、重复规划一致性、平滑碰撞检查和动态地图上的九种配置。完整方法到达目标并停车，仿真时长 128.2 s，行驶 45.7965 m，最小采样净距 0.0917 m。关闭 A* 先验的该次试验发生碰撞。详见 verification/octave_seed42.csv。完整 50 次批量统计和 MATLAB 图形导出未在此次环境中运行。

代码采用 MIT 许可。作者归属及论文图表来源应在再分发时保留。此前的 ROS 2 演示包可独立使用，本仓库的论文实验通过上述 MATLAB 入口执行。
