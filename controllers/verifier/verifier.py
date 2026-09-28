# -*- coding: utf-8 -*-
"""
验证机器人：GPS 定位 + 里程计航向，自动跑完双层迷宫
====================================================
路线由 tools/generate_double_deck.py 预生成（route_data.py）。

控制策略（v3，偏航角速度控制）：
  - 航向 est 由左右轮速差积分（原地/行进中都有效）
  - 期望偏航角速度 w = clamp(K_YAW · 航向误差, ±YAW_MAX)，温和转向不打滑
  - vl = V − w·轮距/2，vr = V + w·轮距/2
  - 位置用 GPS 绝对定位，无累积漂移
历史教训：v2 用全速差原地转（偏航 2000°/s），轮子打滑导致失控。
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

V = 2.5         # 轮速 rad/s，线速度 0.10 m/s（坡上保守）
K_YAW = 3.0     # 航向比例增益
YAW_MAX = 2.0   # 最大偏航角速度 rad/s（约 115°/s，温和）
THRESH = 0.18   # 路点到达半径 m
TRACK = 0.14    # 左右轮距 m
TIMEOUT = 600.0  # 秒，超时保护


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


idx = 0
est = 0.0            # 初始朝东（+X），与世界文件一致
vl_cmd = vr_cmd = 0.0
t0 = robot.getTime()

print('验证机器人启动：共 %d 个路点' % len(WAYPOINTS))

while robot.step(timestep) != -1:
    # 里程计航向：用上一步指令轮速差积分
    est += (vr_cmd - vl_cmd) / TRACK * dt

    p = gps.getValues()
    x, y = p[0], p[1]

    if idx >= len(WAYPOINTS):
        for w in left + right:
            w.setVelocity(0.0)
        print('✅ 验证通过：到达二层终点！全程用时 %.1f 秒'
              % (robot.getTime() - t0))
        break

    if robot.getTime() - t0 > TIMEOUT:
        for w in left + right:
            w.setVelocity(0.0)
        print('❌ 超时未完成：卡在路点 %d/%d (%.2f, %.2f)'
              % (idx + 1, len(WAYPOINTS), x, y))
        break

    tx, ty = WAYPOINTS[idx]
    d = math.hypot(tx - x, ty - y)
    if d < THRESH:
        print('  路点 %d/%d ✓  (%.2f, %.2f)' % (idx + 1, len(WAYPOINTS), tx, ty))
        idx += 1
        continue

    e = wrap(math.atan2(ty - y, tx - x) - est)
    w = clamp(K_YAW * e, -YAW_MAX, YAW_MAX)
    vl_cmd = V - w * TRACK / 2
    vr_cmd = V + w * TRACK / 2

    left[0].setVelocity(vl_cmd)
    left[1].setVelocity(vl_cmd)
    right[0].setVelocity(vr_cmd)
    right[1].setVelocity(vr_cmd)
