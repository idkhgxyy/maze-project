#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
5×5 迷宫生成器（深度优先 / DFS 算法）
====================================
功能：
  1. 用 DFS 生成"完美迷宫"——任意两格之间恰好有一条路径（主路径唯一），天然带死胡同；
  2. 死胡同格子里放彩色方块障碍物（主路径完全畅通）；
  3. 输出 worlds/maze.wbt —— Webots R2023b 兼容，纯基础节点，零联网依赖；
  4. 输出 docs/maze_layout.png —— 俯视布局图，可直接用于 PPT 讲解。

版本迭代：换一个 --seed 就能生成一张新迷宫，配合 git 管理每一版地图。

用法：
  python3 tools/generate_maze.py                        # 默认 7×7，seed=37
  python3 tools/generate_maze.py --seed 7               # 换迷宫
  python3 tools/generate_maze.py --size 9 --seed 10     # 换规模 + 换迷宫
"""
import argparse
import os
import random

import matplotlib
matplotlib.use('Agg')
from matplotlib import font_manager
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon

# ---------- 迷宫规格（与 README 保持一致） ----------
N = 7              # 每边格子数（可被 --size 覆盖）
CELL = 0.5         # 通道宽 m
WALL_T = 0.02      # 墙厚 m
WALL_H = 0.2       # 墙高 m
HALF = N * CELL / 2.0        # 场地半宽 = 1.25 m
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

OBSTACLE_COLORS = [
    (0.80, 0.30, 0.35),   # 莓红
    (0.25, 0.50, 0.75),   # 海蓝
    (0.55, 0.40, 0.75),   # 紫罗兰
    (0.90, 0.65, 0.20),   # 琥珀
    (0.20, 0.65, 0.60),   # 松石绿
    (0.45, 0.50, 0.55),   # 岩灰
]
OBST_SIZE = 0.15   # 障碍物边长 m


def cell_center(i, j):
    """格子 (列i, 行j) 的世界坐标中心，i 从西向东，j 从南向北。"""
    return -HALF + CELL / 2 + CELL * i, -HALF + CELL / 2 + CELL * j


def generate_maze(seed):
    """DFS 挖墙生成完美迷宫。walls[j][i] 记录每格四面墙是否保留。"""
    rng = random.Random(seed)
    visited = [[False] * N for _ in range(N)]
    walls = [[{'N': True, 'S': True, 'E': True, 'W': True}
              for _ in range(N)] for _ in range(N)]
    stack = [(0, 0)]
    visited[0][0] = True
    while stack:
        i, j = stack[-1]
        neighbors = []
        for di, dj, cur, opp in ((0, 1, 'N', 'S'), (0, -1, 'S', 'N'),
                                 (1, 0, 'E', 'W'), (-1, 0, 'W', 'E')):
            ni, nj = i + di, j + dj
            if 0 <= ni < N and 0 <= nj < N and not visited[nj][ni]:
                neighbors.append((ni, nj, cur, opp))
        if not neighbors:
            stack.pop()
            continue
        ni, nj, cur, opp = rng.choice(neighbors)
        walls[j][i][cur] = False
        walls[nj][ni][opp] = False
        visited[nj][ni] = True
        stack.append((ni, nj))
    return walls, rng


def find_dead_ends(walls):
    """恰好三面墙的格子 = 死胡同。排除起点(0,0)和终点(N-1,N-1)。"""
    dead = []
    for j in range(N):
        for i in range(N):
            if (i, j) in ((0, 0), (N - 1, N - 1)):
                continue
            if sum(walls[j][i].values()) == 3:
                dead.append((i, j))
    return dead


def collect_wall_segments(walls):
    """收集所有墙段：返回 (x, y, kind, length)，kind='V' 竖墙 / 'H' 横墙。"""
    segs = []
    L = 2 * HALF + 2 * WALL_T          # 外墙长度（含角部搭接）
    segs.append((0, HALF, 'H', L))
    segs.append((0, -HALF, 'H', L))
    segs.append((HALF, 0, 'V', L))
    segs.append((-HALF, 0, 'V', L))
    for i in range(N - 1):
        for j in range(N):
            if walls[j][i]['E']:       # (i,j) 与 (i+1,j) 之间
                segs.append((-HALF + CELL * (i + 1),
                             cell_center(i, j)[1], 'V', CELL + 2 * WALL_T))
    for j in range(N - 1):
        for i in range(N):
            if walls[j][i]['N']:       # (i,j) 与 (i,j+1) 之间
                segs.append((cell_center(i, j)[0],
                             -HALF + CELL * (j + 1), 'H', CELL + 2 * WALL_T))
    return segs


def ascii_layout(walls):
    """控制台 ASCII 预览。"""
    lines = []
    for j in reversed(range(N)):                    # 北在上
        top = '+'
        for i in range(N):
            top += '---+' if walls[j][i]['N'] else '   +'
        lines.append(top)
        mid = ''
        for i in range(N):
            mid += '|' if walls[j][i]['W'] else ' '
            mid += ' S ' if (i, j) == (0, 0) else (
                   ' G ' if (i, j) == (N - 1, N - 1) else '   ')
        mid += '|'
        lines.append(mid)
    lines.append('+' + '---+' * N)
    return '\n'.join(lines)


def build_wbt(walls, dead_ends, rng):
    """拼装 R2023b 世界文件字符串（纯基础节点，零 EXTERNPROTO）。"""
    segs = collect_wall_segments(walls)

    parts = []
    a = parts.append
    a('#VRML_SIM R2023b utf8\n')
    a('WorldInfo {\n  basicTimeStep 16\n}\n')
    a('# 浅色天空背景（投影仪友好）\n')
    a('Background {\n  skyColor [ 0.78 0.84 0.88 ]\n}\n')
    a('Viewpoint {\n  orientation -0.5774 0.5774 0.5774 2.0944\n'
      '  position 0 0 3.2\n}\n')
    a('DirectionalLight {\n  direction -0.3 0.3 -1\n  intensity 1.4\n'
      '  ambientIntensity 0.4\n}\n')

    # 地板
    a('''DEF FLOOR Solid {
  children [
    DEF FLOOR_SHAPE Shape {
      appearance PBRAppearance {
        baseColor 0.92 0.88 0.82
        roughness 1
        metalness 0
      }
      geometry Plane {
        size 9 9
      }
    }
  ]
  name "floor"
  boundingObject USE FLOOR_SHAPE
  locked TRUE
}
''')

    # 起点 / 终点色块（纯视觉，无碰撞，机器人可以直接碾过去）
    sx, sy = cell_center(0, 0)
    fx, fy = cell_center(N - 1, N - 1)
    for x, y, color, defname, label in (
            (sx, sy, (0.15, 0.75, 0.30), 'START_PAD', '起点'),
            (fx, fy, (1.0, 0.45, 0.05), 'FINISH_PAD', '终点')):
        a('DEF %s Solid {\n  translation %s %s 0.001\n  children [\n'
          '    Shape {\n      appearance PBRAppearance {\n'
          '        baseColor %s %s %s\n        roughness 1\n'
          '        metalness 0\n      }\n      geometry Box {\n'
          '        size %s %s 0.002\n      }\n    }\n  ]\n'
          '  name "%s"\n}\n'
          % (defname, x, y, color[0], color[1], color[2],
             CELL, CELL, label))

    # 迷宫墙（%d 段）
    a('# 迷宫墙（%d 段）\n' % len(segs))
    for k, (x, y, kind, length) in enumerate(segs):
        if kind == 'V':
            size = '%s %s %s' % (WALL_T, length, WALL_H)
        else:
            size = '%s %s %s' % (length, WALL_T, WALL_H)
        a('''DEF WALL_%d Solid {
  translation %s %s %s
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0.18 0.43 0.42
        roughness 1
        metalness 0
      }
      geometry Box {
        size %s
      }
    }
  ]
  boundingObject Box {
    size %s
  }
  name "wall_%d"
}
''' % (k + 1, x, y, WALL_H / 2, size, size, k + 1))

    # 死胡同障碍物
    a('# 死胡同障碍物（%d 个，主路径无障碍）\n' % len(dead_ends))
    for k, (i, j) in enumerate(dead_ends):
        x, y = cell_center(i, j)
        c = OBSTACLE_COLORS[k % len(OBSTACLE_COLORS)]
        a('''DEF OBSTACLE_%d Solid {
  translation %s %s %s
  children [
    Shape {
      appearance PBRAppearance {
        baseColor %s %s %s
        roughness 1
        metalness 0
      }
      geometry Box {
        size %s %s %s
      }
    }
  ]
  boundingObject Box {
    size %s %s %s
  }
  name "obstacle_%d"
}
''' % (k + 1, x, y, OBST_SIZE / 2, c[0], c[1], c[2],
       OBST_SIZE, OBST_SIZE, OBST_SIZE,
       OBST_SIZE, OBST_SIZE, OBST_SIZE, k + 1))

    # 占位四轮机器人（纯基础节点，R2023b 原生支持，无任何联网依赖）
    a(('''# 占位四轮机器人（起点朝东，等待机器人组替换/改造）
DEF PLACEHOLDER_CAR Robot {
  translation %s %s 0.05
  rotation 0 0 1 0
  children [
    DEF CAR_BODY Shape {
      appearance PBRAppearance {
        baseColor 0.20 0.40 0.80
        roughness 1
        metalness 0
      }
      geometry Box {
        size 0.2 0.1 0.05
      }
    }
    DEF WHEEL1 HingeJoint {
      jointParameters HingeJointParameters {
        axis 0 1 0
        anchor 0.05 0.06 0
      }
      device [
        RotationalMotor {
          name "wheel1"
        }
      ]
      endPoint Solid {
        translation 0.05 0.06 0
        rotation 1 0 0 1.5708
        children [
          DEF WHEEL Shape {
            appearance PBRAppearance {
              baseColor 0.20 0.20 0.20
              roughness 1
              metalness 0
            }
            geometry Cylinder {
              height 0.02
              radius 0.04
              subdivision 24
            }
          }
        ]
        boundingObject USE WHEEL
        physics Physics {
        }
      }
    }
    DEF WHEEL2 HingeJoint {
      jointParameters HingeJointParameters {
        axis 0 1 0
        anchor 0.05 -0.06 0
      }
      device [
        RotationalMotor {
          name "wheel2"
        }
      ]
      endPoint Solid {
        translation 0.05 -0.06 0
        rotation 1 0 0 1.5708
        children [
          USE WHEEL
        ]
        name "solid(1)"
        boundingObject USE WHEEL
        physics Physics {
        }
      }
    }
    DEF WHEEL3 HingeJoint {
      jointParameters HingeJointParameters {
        axis 0 1 0
        anchor -0.05 0.06 0
      }
      device [
        RotationalMotor {
          name "wheel3"
        }
      ]
      endPoint Solid {
        translation -0.05 0.06 0
        rotation 1 0 0 1.5708
        children [
          USE WHEEL
        ]
        name "solid(2)"
        boundingObject USE WHEEL
        physics Physics {
        }
      }
    }
    DEF WHEEL4 HingeJoint {
      jointParameters HingeJointParameters {
        axis 0 1 0
        anchor -0.05 -0.06 0
      }
      device [
        RotationalMotor {
          name "wheel4"
        }
      ]
      endPoint Solid {
        translation -0.05 -0.06 0
        rotation 1 0 0 1.5708
        children [
          USE WHEEL
        ]
        name "solid(3)"
        boundingObject USE WHEEL
        physics Physics {
        }
      }
    }
  ]
  boundingObject USE CAR_BODY
  physics Physics {
    density -1
    mass 1
  }
  controller "placeholder_car"
}
''') % (sx, sy))
    return '\n'.join(parts)


def draw_layout_png(walls, dead_ends, seed, out_path):
    """俯视布局图。"""
    font_path = '/System/Library/Fonts/STHeiti Medium.ttc'
    if os.path.exists(font_path):
        font_manager.fontManager.addfont(font_path)
        prop = font_manager.FontProperties(fname=font_path)
        plt.rcParams['font.family'] = prop.get_name()
        plt.rcParams['axes.unicode_minus'] = False

    fig, ax = plt.subplots(figsize=(8.5, 8.5))
    ax.set_xlim(-(HALF + 0.35), HALF + 0.35)
    ax.set_ylim(-(HALF + 0.35), HALF + 0.35)
    ax.set_aspect('equal')

    # 地板与起终点（按当前规模自动定位）
    sx0, sy0 = cell_center(0, 0)
    fx0, fy0 = cell_center(N - 1, N - 1)
    ax.add_patch(Rectangle((-HALF, -HALF), 2 * HALF, 2 * HALF,
                           facecolor='#ece4d6', edgecolor='none'))
    ax.add_patch(Rectangle((sx0 - CELL / 2, sy0 - CELL / 2), CELL, CELL,
                           facecolor='#b7e8c0', edgecolor='none'))
    ax.add_patch(Rectangle((fx0 - CELL / 2, fy0 - CELL / 2), CELL, CELL,
                           facecolor='#ffd0a8', edgecolor='none'))

    # 障碍物
    for k, (i, j) in enumerate(dead_ends):
        x, y = cell_center(i, j)
        c = OBSTACLE_COLORS[k % len(OBSTACLE_COLORS)]
        ax.add_patch(Rectangle((x - OBST_SIZE / 2, y - OBST_SIZE / 2),
                               OBST_SIZE, OBST_SIZE,
                               facecolor=c, edgecolor='black', lw=0.8))
        ax.text(x, y, '障', ha='center', va='center',
                fontsize=8, color='white')

    # 墙
    for x, y, kind, length in collect_wall_segments(walls):
        if kind == 'V':
            ax.add_patch(Rectangle((x - WALL_T / 2, y - length / 2),
                                   WALL_T, length,
                                   facecolor='#2e6e6a', edgecolor='none'))
        else:
            ax.add_patch(Rectangle((x - length / 2, y - WALL_T / 2),
                                   length, WALL_T,
                                   facecolor='#2e6e6a', edgecolor='none'))

    # 占位机器人（起点，车头朝东）
    rx, ry = cell_center(0, 0)
    ax.add_patch(Polygon([(rx + 0.09, ry), (rx - 0.06, ry + 0.06),
                          (rx - 0.06, ry - 0.06)],
                         closed=True, facecolor='#2a56c6',
                         edgecolor='black', lw=0.8, zorder=5))
    ax.text(rx, ry - 0.16, '占位机器人', ha='center', va='top', fontsize=9)

    fx, fy = cell_center(N - 1, N - 1)
    ax.text(rx, ry + CELL / 2 + 0.03, '起点', ha='center', va='bottom', fontsize=10)
    ax.text(fx, fy + CELL / 2 + 0.03, '终点（橙色区）', ha='center', va='bottom', fontsize=10)
    ax.text(0, HALF + 0.17, '%d×%d 迷宫俯视布局图（seed=%d，通道宽 0.5 m）' % (N, N, seed),
            ha='center', va='bottom', fontsize=13)
    ax.text(0, -(HALF + 0.13), '尺寸：场地 %g×%g m　墙高 0.2 m　障碍方块 0.15 m' % (2 * HALF, 2 * HALF),
            ha='center', va='top', fontsize=9, color='#555555')

    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def main():
    global N, HALF
    parser = argparse.ArgumentParser(description='DFS 迷宫生成器')
    parser.add_argument('--size', type=int, default=7,
                        help='每边格子数（默认 7）')
    parser.add_argument('--seed', type=int, default=37,
                        help='随机种子，换数字即换迷宫')
    args = parser.parse_args()
    N = args.size
    HALF = N * CELL / 2.0

    walls, rng = generate_maze(args.seed)
    dead_ends = find_dead_ends(walls)

    print(ascii_layout(walls))
    print('\n死胡同格子（放置障碍物）：%s' %
          ', '.join('(%d,%d)' % d for d in dead_ends))
    print('起点 (0,0) 左下角，终点 (%d,%d) 右上角' % (N - 1, N - 1))

    os.makedirs(os.path.join(ROOT, 'worlds'), exist_ok=True)
    os.makedirs(os.path.join(ROOT, 'docs'), exist_ok=True)
    wbt_path = os.path.join(ROOT, 'worlds', 'maze.wbt')
    with open(wbt_path, 'w', encoding='utf-8') as f:
        f.write(build_wbt(walls, dead_ends, rng))
    print('已写入 %s' % wbt_path)

    png_path = os.path.join(ROOT, 'docs', 'maze_layout.png')
    draw_layout_png(walls, dead_ends, args.seed, png_path)
    print('已写入 %s' % png_path)


if __name__ == '__main__':
    main()
