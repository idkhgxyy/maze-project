# -*- coding: utf-8 -*-
"""
验证机器人：GPS + 里程计航向，自动跑完双层迷宫
================================================
路线由 tools/generate_double_deck.py 预生成（route_data.py）。

航向估计：用左右轮速差做里程计积分（原地转向时也有效），
位置用 GPS 绝对定位（无累积漂移）。两者互补：
  - 只用 GPS 位移估航向 → 原地转向时航向永远不更新 → 死循环打转（已修复）
控制：先原地转到目标方位 ±29° 以内，再比例转向前进。
"""
import math
import os
import sys

from controller import Robot

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from route_data import WAYPOINTS

robot = Robot()
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0

gps = robot.getDevice('gps')
gps.enable(timestep)

left = [robot.getDevice('wheel1'), robot.getDevice('wheel3')]
right = [robot.getDevice('wheel2'), robot.getDevice('wheel4')]
for w in left + right:
    w.setPosition(float('inf'))   # 速度模式

V = 2.5         # rad/s，线速度 0.10 m/s（坡上保守）
K = 1.8         # 转向比例增益
THRESH = 0.18   # 路点到达半径 m
SPIN_TH = 0.5   # 偏差超过 0.5 rad 先原地转向
TRACK = 0.14    # 左右轮距 m


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


idx = 0
est = 0.0            # 初始朝东（+X），与世界文件一致
vl_cmd = vr_cmd = 0.0
t0 = robot.getTime()

print('验证机器人启动：共 %d 个路点' % len(WAYPOINTS))

while robot.step(timestep) != -1:
    # 里程计航向：用上一步指令轮速积分（原地转向时依然有效）
    est = est + (vr_cmd - vl_cmd) / TRACK * dt

    p = gps.getValues()
    x, y = p[0], p[1]

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
    if abs(e) > SPIN_TH:
        vl_cmd, vr_cmd = (-V, V) if e > 0 else (V, -V)   # 原地转向
    else:
        vl_cmd = max(-2 * V, min(2 * V, V * (1 - K * e)))
        vr_cmd = max(-2 * V, min(2 * V, V * (1 + K * e)))

    left[0].setVelocity(vl_cmd)
    left[1].setVelocity(vl_cmd)
    right[0].setVelocity(vr_cmd)
    right[1].setVelocity(vr_cmd)
