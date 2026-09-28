# Webots 迷宫场景（maze-project）

7×7 自主走迷宫挑战场地。由深度优先（DFS）算法生成，保证从起点到终点**一定有唯一主路径**，并带死胡同分支；死胡同内放置障碍方块，**主路径完全畅通**。主路径约 19.5 m，按典型小车速度（0.24 m/s）加探索弯路，全程约 1.5~2 分钟。

## 环境要求

- Webots **R2023b 或更高版本**（在 R2023b 上原生兼容，无需联网下载任何资源）
- 世界文件仅使用 Webots 基础节点（Solid / Box / Cylinder / HingeJoint 等），零 EXTERNPROTO 依赖

## 使用方法

1. 下载本仓库（网页 Code → Download ZIP，或 `git clone`）
2. 用 Webots 打开 `worlds/maze.wbt`
3. 场景内蓝色小车是**占位机器人**，停在起点（绿色区域），控制器当前为静止状态
4. 点击 ▶ 运行确认场景正常，然后开始改造机器人与控制器

## 迷宫规格

| 项目 | 数值 |
|---|---|
| 格子 | 7×7 |
| 通道宽 | 0.5 m |
| 场地 | 3.5 × 3.5 m |
| 墙高 / 墙厚 | 0.2 m / 0.02 m |
| 障碍方块 | 0.15 m，只放在死胡同 |
| 起点 | 左下角绿色区域，车头朝东（+X） |
| 终点 | 右上角橙色区域，到达即完成 |

俯视布局图见 `docs/maze_layout.png`（seed=37）。

## 任务说明（给机器人组）

- 目标：机器人从起点出发，自主穿越迷宫，到达橙色终点区域
- 占位机器人：0.2×0.1×0.05 m 车身 + 4 个半径 0.04 m 轮子，差速驱动
  - 左轮 = `wheel1`(前) + `wheel3`(后)，右轮 = `wheel2`(前) + `wheel4`(后)
  - 控制器在 `controllers/placeholder_car/placeholder_car.py`，已按速度模式初始化好四个电机
- 可以直接替换/改造机器人（加传感器、改结构都行），迷宫墙体和障碍物不要动
- 想知道整张地图？看 `docs/maze_layout.png`；只想"盲走"也可以不提前看

## 迷宫版本迭代（地图组）

```bash
python3 tools/generate_maze.py --seed 37              # 当前版本（7×7, seed=37）
python3 tools/generate_maze.py --seed 20              # 换种子生成新迷宫（会覆盖 worlds/maze.wbt）
python3 tools/generate_maze.py --size 9 --seed 10     # 换规模 + 换迷宫
```

流程：生成 → 检查 `docs/maze_layout.png` → Webots 打开验证 → git commit 记录这一版。

## 目录结构

```
maze-project/
├── worlds/maze.wbt                    # Webots 世界（R2023b 兼容）
├── controllers/placeholder_car/       # 占位四轮车控制器
├── tools/generate_maze.py             # DFS 迷宫生成器
├── docs/maze_layout.png               # 俯视布局图
├── README.md
└── .gitignore
```
