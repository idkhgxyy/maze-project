# -*- coding: utf-8 -*-
"""
验证机器人：GPS 循迹自动跑完双层迷宫
====================================
路线由 tools/generate_double_deck.py 预生成（route_data.py）。
原理：GPS 实时定位 → 朝下一个路点做比例转向（纯追踪）；
航向角由 GPS 位移估计，无需罗盘。到达全部路点即验证"迷宫可通行"。
"""
import math
import os
import sys

from controller import Robot

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from route_data import WAYPOINTS

robot = Robot()
timestep = int(robot.getBasicTimeStep())

gps = robot.getDevice('gps')
gps.enable(timestep)

left = [robot.getDevice('wheel1'), robot.getDevice('wheel3')]
right = [robot.getDevice('wheel2'), robot.getDevice('wheel4')]
for w in left + right:
    w.setPosition(float('inf'))   # 速度模式

V = 2.5        # rad/s，线速度 0.10 m/s（坡上保守）
K = 2.2        # 转向比例增益
THRESH = 0.18  # 路点到达半径 m


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


idx = 0
est = 0.0            # 初始朝东（+X），与世界文件一致
prev = None
t0 = robot.getTime()

print('验证机器人启动：共 %d 个路点（一层 %d → 坡道 → 二层 → 终点）'
      % (len(WAYPOINTS), len(WAYPOINTS)))

while robot.step(timestep) != -1:
    p = gps.getValues()
    x, y = p[0], p[1]

    if prev is not None:
        dx, dy = x - prev[0], y - prev[1]
        if math.hypot(dx, dy) > 0.01:
            est = math.atan2(dy, dx)   # 用位移估计航向
    prev = (x, y)

    if idx >= len(WAYPOINTS):
        for w in left + right:
            w.setVelocity(0.0)
        print('✅ 验证通过：到达二层终点！全程用时 %.1f 秒'
              % (robot.getTime() - t0))
        break

    tx, ty = WAYPOINTS[idx]
    d = math.hypot(tx - x, ty - y)
    if d < THRESH:
        print('  路点 %d/%d ✓  (%.2f, %.2f)' % (idx + 1, len(WAYPOINTS), tx, ty))
        idx += 1
        continue

    e = wrap(math.atan2(ty - y, tx - x) - est)
    vl = max(-2 * V, min(2 * V, V * (1 - K * e)))
    vr = max(-2 * V, min(2 * V, V * (1 + K * e)))
    left[0].setVelocity(vl)
    left[1].setVelocity(vl)
    right[0].setVelocity(vr)
    right[1].setVelocity(vr)
