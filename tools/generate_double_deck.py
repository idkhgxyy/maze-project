#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双层立体迷宫生成器（验证版，double-deck 分支专用）
==================================================
结构：
  - 一、二层各一个 5×5 DFS 迷宫（2.5×2.5m），层高 0.30m
  - 二层楼板：整块 2.5×2.5 薄板（无洞，杜绝坠落），四角装饰柱
  - 层间连接：楼外东侧"斜坡 + 高架栈桥"
      一层东墙 row0 开闸口 → 地面 → 北向缓坡（水平 1.20m 抬升 0.30m ≈ 14°）
      → 坡顶接栈桥（与楼板顶面齐平）→ 二层东墙 row2 闸口 → 二层迷宫
  - 路线：工具自动为两层各生成 DFS 迷宫并求最短路，写入验证机器人路点

输出：
  worlds/maze_double_deck.wbt         （R2023b 兼容，零 EXTERNPROTO）
  controllers/verifier/route_data.py  （验证机器人 GPS 循迹路点）
  docs/maze_double_deck_layout.png    （两层布局图）

用法：python3 tools/generate_double_deck.py
"""
import math
import os
from collections import deque

import matplotlib
matplotlib.use('Agg')
from matplotlib import font_manager
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrow

import generate_maze as gm   # 复用单层 DFS 迷宫算法

# ---------- 规格 ----------
N, CELL, HALF = 5, 0.5, 1.25
gm.N, gm.HALF = N, HALF
WALL_T, WALL_H = 0.02, 0.18
DECK_TOP, DECK_T = 0.30, 0.02          # 二层楼板顶面高度 / 板厚
RAMP_W = 0.40
RISE, RUN = DECK_TOP, 1.20             # 坡度 ≈ 14°
RAMP_LEN = math.hypot(RISE, RUN)
RAMP_ANG = math.atan2(RISE, RUN)
RAMP_X, RAMP_Y0 = 1.5, -0.95           # 坡底位置（坡顶 y = RAMP_Y0 + RUN = 0.25）
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

TEAL = (0.18, 0.43, 0.42)
CREAM = (0.92, 0.88, 0.82)
GRAY = (0.55, 0.55, 0.58)
ORANGE = (1.0, 0.45, 0.05)
GREEN = (0.15, 0.75, 0.30)
CAR = (0.85, 0.45, 0.10)
DIRS = (('N', 0, 1), ('S', 0, -1), ('E', 1, 0), ('W', -1, 0))


def cc(i, j):
    return (-1.0 + 0.5 * i, -1.0 + 0.5 * j)


def shortest_path(walls, start, goal):
    dist = {start: 0}
    q = deque([start])
    while q:
        cur = q.popleft()
        if cur == goal:
            break
        i, j = cur
        for d, ci, cj in DIRS:
            if not walls[j][i][d]:
                nxt = (i + ci, j + cj)
                if nxt not in dist:
                    dist[nxt] = dist[cur] + 1
                    q.append(nxt)
    if goal not in dist:
        return None
    path, cur = [goal], goal
    while cur != start:
        i, j = cur
        for d, ci, cj in DIRS:
            if not walls[j][i][d]:
                nxt = (i + ci, j + cj)
                if dist.get(nxt, 10 ** 9) == dist[cur] - 1:
                    path.append(nxt)
                    cur = nxt
                    break
    path.reverse()
    return path


def pick_floor(seed_from, start, goal):
    for s in range(seed_from, 300):
        walls, _ = gm.generate_maze(s)
        p = shortest_path(walls, start, goal)
        if p and len(p) >= 9:
            return s, walls, p
    raise RuntimeError('找不到合适种子')


def solid(name, x, y, z, sx, sy, sz, color, rot=None):
    r = ('  rotation %s\n' % rot) if rot else ''
    return ('''DEF %s Solid {
  translation %s %s %s
%s  children [
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
  name "%s"
}
''' % (name, x, y, z, r, color[0], color[1], color[2],
       sx, sy, sz, sx, sy, sz, name.lower()))


def internal_walls(walls, zc, tag):
    out, k = [], 0
    for i in range(N - 1):
        for j in range(N):
            if walls[j][i]['E']:
                out.append(solid('%s_V%d' % (tag, k),
                                 -1.25 + 0.5 * (i + 1), -1.0 + 0.5 * j, zc,
                                 WALL_T, CELL + 2 * WALL_T, WALL_H, TEAL))
                k += 1
    for j in range(N - 1):
        for i in range(N):
            if walls[j][i]['N']:
                out.append(solid('%s_H%d' % (tag, k),
                                 -1.0 + 0.5 * i, -1.25 + 0.5 * (j + 1), zc,
                                 CELL + 2 * WALL_T, WALL_T, WALL_H, TEAL))
                k += 1
    return out


def outer_walls(floor, tag):
    """东西南北外墙；东墙按楼层留闸口（1层row0 / 2层row2）。"""
    zc = WALL_H / 2 if floor == 1 else DECK_TOP + WALL_H / 2
    L = 2 * HALF + 2 * WALL_T
    out = [
        solid('%s_N' % tag, 0, HALF, zc, L, WALL_T, WALL_H, TEAL),
        solid('%s_S' % tag, 0, -HALF, zc, L, WALL_T, WALL_H, TEAL),
        solid('%s_W' % tag, -HALF, 0, zc, WALL_T, L, WALL_H, TEAL),
    ]
    if floor == 1:      # 闸口 y∈[-1.25,-0.75]
        out.append(solid('%s_E' % tag, HALF, 0.25, zc,
                         WALL_T, 2.0, WALL_H, TEAL))
    else:               # 闸口 y∈[0.25,0.75]（row3）
        out.append(solid('%s_E1' % tag, HALF, -0.5, zc,
                         WALL_T, 1.5, WALL_H, TEAL))
        out.append(solid('%s_E2' % tag, HALF, 1.0, zc,
                         WALL_T, 0.5, WALL_H, TEAL))
    return out


def build_wbt(w1, w2, waypoints):
    lines = []
    a = lines.append
    a('#VRML_SIM R2023b utf8\n')
    a('WorldInfo {\n  basicTimeStep 16\n}\n')
    a('Viewpoint {\n  orientation 0.7431 0.3078 0.5944 1.2169\n'
      '  position 3.4 -3.4 3.4\n}\n')
    a('Background {\n  skyColor [ 0.78 0.84 0.88 ]\n}\n')
    a('DirectionalLight {\n  direction -0.4 0.4 -1\n  intensity 1.4\n'
      '  ambientIntensity 0.4\n}\n')

    a('''DEF FLOOR Solid {
  children [
    DEF FLOOR_GEOM Shape {
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
  boundingObject USE FLOOR_GEOM
  locked TRUE
}
''')

    a('# ===== 一层迷宫（z: 0 ~ 0.18）=====\n')
    for s in outer_walls(1, 'F1') + internal_walls(w1, WALL_H / 2, 'F1I'):
        a(s)

    a('# ===== 二层楼板 / 柱 / 斜坡 / 栈桥 =====\n')
    a(solid('DECK', 0, 0, DECK_TOP - DECK_T / 2, 2.5, 2.5, DECK_T, CREAM))
    k = 0
    for cx in (-1.18, 1.18):
        for cy in (-1.18, 1.18):
            k += 1
            a(solid('COLUMN_%d' % k, cx, cy, (DECK_TOP - DECK_T) / 2,
                    0.08, 0.08, DECK_TOP - DECK_T, GRAY))
    # 斜坡（绕 X 轴抬升 RAMP_ANG，坡面从 (1.5,-0.95,0) 到 (1.5,0.25,0.30)）
    rot = '1 0 0 %s' % round(RAMP_ANG, 4)
    cyc, czc = RAMP_Y0 + RUN / 2, RISE / 2
    a(solid('RAMP', RAMP_X, cyc - 0.003, czc - 0.0097,
            RAMP_W, round(RAMP_LEN, 4), DECK_T, CREAM, rot))
    a(solid('RAMP_RAIL_W', RAMP_X - RAMP_W / 2 - 0.01, cyc, czc + 0.04,
            0.02, round(RAMP_LEN, 4), 0.08, TEAL, rot))
    a(solid('RAMP_RAIL_E', RAMP_X + RAMP_W / 2 + 0.01, cyc, czc + 0.04,
            0.02, round(RAMP_LEN, 4), 0.08, TEAL, rot))
    # 栈桥（顶面与楼板齐平），东/北沿加栏杆
    a(solid('BRIDGE', 1.4, 0.1, DECK_TOP - DECK_T / 2,
            0.5, 0.8, DECK_T, CREAM))
    a(solid('BRIDGE_RAIL_E', 1.66, 0.1, DECK_TOP + 0.04,
            0.02, 0.8, 0.08, TEAL))
    a(solid('BRIDGE_RAIL_N', 1.4, 0.51, DECK_TOP + 0.04,
            0.5, 0.02, 0.08, TEAL))

    a('# ===== 二层迷宫（楼板上）=====\n')
    for s in outer_walls(2, 'F2') + internal_walls(w2, DECK_TOP + WALL_H / 2, 'F2I'):
        a(s)

    # 起点 / 终点色块
    sx, sy = cc(0, 0)
    fx, fy = cc(0, 4)
    a('''DEF START_PAD Solid {
  translation %s %s 0.001
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0.15 0.75 0.30
        roughness 1
        metalness 0
      }
      geometry Box {
        size 0.5 0.5 0.002
      }
    }
  ]
  name "start_pad"
}
''' % (sx, sy))
    a('''DEF FINISH_PAD Solid {
  translation %s %s %s
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 1.0 0.45 0.05
        roughness 1
        metalness 0
      }
      geometry Box {
        size 0.5 0.5 0.002
      }
    }
  ]
  name "finish_pad"
}
''' % (fx, fy, DECK_TOP + 0.001))

    a('# ===== 验证机器人（GPS 循迹，路线见 route_data.py）=====\n')
    a('''DEF VERIFIER Robot {
  translation -1.0 -1.0 0.05
  rotation 0 0 1 0
  children [
    GPS {
      translation 0 0 0.06
      name "gps"
    }
    DEF BODY Shape {
      appearance PBRAppearance {
        baseColor 0.85 0.45 0.10
        roughness 1
        metalness 0
      }
      geometry Box {
        size 0.18 0.14 0.05
      }
    }
    DEF W1 HingeJoint {
      jointParameters HingeJointParameters {
        axis 0 1 0
        anchor 0.06 0.07 0
      }
      device [
        RotationalMotor {
          name "wheel1"
          maxTorque 10
        }
      ]
      endPoint Solid {
        translation 0.06 0.07 0
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
    DEF W2 HingeJoint {
      jointParameters HingeJointParameters {
        axis 0 1 0
        anchor 0.06 -0.07 0
      }
      device [
        RotationalMotor {
          name "wheel2"
          maxTorque 10
        }
      ]
      endPoint Solid {
        translation 0.06 -0.07 0
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
    DEF W3 HingeJoint {
      jointParameters HingeJointParameters {
        axis 0 1 0
        anchor -0.06 0.07 0
      }
      device [
        RotationalMotor {
          name "wheel3"
          maxTorque 10
        }
      ]
      endPoint Solid {
        translation -0.06 0.07 0
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
    DEF W4 HingeJoint {
      jointParameters HingeJointParameters {
        axis 0 1 0
        anchor -0.06 -0.07 0
      }
      device [
        RotationalMotor {
          name "wheel4"
          maxTorque 10
        }
      ]
      endPoint Solid {
        translation -0.06 -0.07 0
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
  boundingObject USE BODY
  physics Physics {
    density -1
    mass 2
  }
  controller "verifier"
}
''')
    return '\n'.join(lines)


def route_file(wps):
    lines = ['# 自动生成：双层迷宫验证路线（tools/generate_double_deck.py）',
             'WAYPOINTS = [']
    for x, y in wps:
        lines.append('    (%.3f, %.3f),' % (x, y))
    lines.append(']')
    return '\n'.join(lines) + '\n'


def draw_panel(ax, walls, gate_ys, title, path, marks):
    for i in range(N - 1):
        for j in range(N):
            if walls[j][i]['E']:
                x = -1.25 + 0.5 * (i + 1)
                y = -1.0 + 0.5 * j
                ax.add_patch(Rectangle((x - WALL_T / 2, y - (CELL + 2 * WALL_T) / 2),
                                       WALL_T, CELL + 2 * WALL_T,
                                       facecolor='#2e6e6a', edgecolor='none'))
    for j in range(N - 1):
        for i in range(N):
            if walls[j][i]['N']:
                x = -1.0 + 0.5 * i
                y = -1.25 + 0.5 * (j + 1)
                ax.add_patch(Rectangle((x - (CELL + 2 * WALL_T) / 2, y - WALL_T / 2),
                                       CELL + 2 * WALL_T, WALL_T,
                                       facecolor='#2e6e6a', edgecolor='none'))
    # 外墙（东墙留闸口）
    ax.add_patch(Rectangle((-1.25, -1.25), 2.5 + WALL_T, WALL_T, facecolor='#2e6e6a'))
    ax.add_patch(Rectangle((-1.25, 1.25), 2.5 + WALL_T, WALL_T, facecolor='#2e6e6a'))
    ax.add_patch(Rectangle((-1.25, -1.25), WALL_T, 2.5 + WALL_T, facecolor='#2e6e6a'))
    y0, y1 = gate_ys
    if y0 > -1.26:      # 闸口南侧墙
        ax.add_patch(Rectangle((1.25, -1.25), WALL_T, y0 + 1.25, facecolor='#2e6e6a'))
    if y1 < 1.26:       # 闸口北侧墙
        ax.add_patch(Rectangle((1.25, y1), WALL_T, 1.25 - y1, facecolor='#2e6e6a'))
    # 路线
    if path:
        px = [cc(i, j)[0] for i, j in path]
        py = [cc(i, j)[1] for i, j in path]
        ax.plot(px, py, '-', color='#d97a2b', lw=3, alpha=0.85, zorder=4)
    for (x, y, color, txt, ha) in marks:
        ax.text(x, y, txt, ha=ha, va='center', fontsize=10, color=color,
                fontweight='bold', zorder=6)
    ax.set_xlim(-1.45, 1.5)
    ax.set_ylim(-1.45, 1.45)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(title, fontsize=12)
    for s in ax.spines.values():
        s.set_visible(False)


def draw_png(w1, p1, w2, p2, out_path):
    font_path = '/System/Library/Fonts/STHeiti Medium.ttc'
    if os.path.exists(font_path):
        font_manager.fontManager.addfont(font_path)
        prop = font_manager.FontProperties(fname=font_path)
        plt.rcParams['font.family'] = prop.get_name()
        plt.rcParams['axes.unicode_minus'] = False
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 7))
    draw_panel(a1, w1, (-1.25, -0.75), '第一层：起点 → 东侧闸口', p1,
               [(cc(0, 0)[0], cc(0, 0)[1], 'green', '起点', 'center'),
                (1.32, -1.0, '#d97a2b', '闸口→', 'left')])
    draw_panel(a2, w2, (0.25, 0.75), '第二层：东侧闸口 → 终点', p2,
               [(cc(4, 3)[0], cc(4, 3)[1], '#d97a2b', '←闸口', 'center'),
                (cc(0, 4)[0], cc(0, 4)[1], 'orangered', '终点', 'center')])
    fig.suptitle('双层立体迷宫布局（每层 5×5，层高 0.30m，外挂斜坡+栈桥连接）',
                 fontsize=13)
    fig.text(0.5, 0.02,
             '连接：一层东闸口 → 北向缓坡(≈14°) → 栈桥 → 二层东闸口　'
             '墙高 0.18m　通道 0.5m',
             ha='center', fontsize=9, color='#555555')
    fig.tight_layout(rect=(0, 0.04, 1, 0.95))
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main():
    s1, w1, p1 = pick_floor(1, (0, 0), (4, 0))       # 一层：起点→东闸口
    s2, w2, p2 = pick_floor(100, (4, 3), (0, 4))     # 二层：东闸口(row3)→终点
    print('一层 seed=%d 路径 %d 格：%s' % (s1, len(p1), p1))
    print('二层 seed=%d 路径 %d 格：%s' % (s2, len(p2), p2))

    wps = [cc(i, j) for i, j in p1]
    wps += [(1.45, -1.0), (1.5, -0.5), (1.5, 0.45)]  # 出闸 → 上坡 → 桥上待转点
    wps += [cc(i, j) for i, j in p2]                 # 二层路线（含闸口格）
    # 相邻去重
    dedup = [wps[0]]
    for w in wps[1:]:
        if w != dedup[-1]:
            dedup.append(w)

    for d in ('worlds', 'docs', 'controllers/verifier'):
        os.makedirs(os.path.join(ROOT, d), exist_ok=True)
    with open(os.path.join(ROOT, 'worlds', 'maze_double_deck.wbt'),
              'w', encoding='utf-8') as f:
        f.write(build_wbt(w1, w2, dedup))
    print('已写入 worlds/maze_double_deck.wbt（%d 个路点）' % len(dedup))
    with open(os.path.join(ROOT, 'controllers/verifier/route_data.py'),
              'w', encoding='utf-8') as f:
        f.write(route_file(dedup))
    print('已写入 controllers/verifier/route_data.py')
    draw_png(w1, p1, w2, p2, os.path.join(ROOT, 'docs', 'maze_double_deck_layout.png'))
    print('已写入 docs/maze_double_deck_layout.png')


if __name__ == '__main__':
    main()
