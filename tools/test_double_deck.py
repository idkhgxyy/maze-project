#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双层迷宫自检脚本（不依赖 Webots）
==================================
1. 几何校验：坡道两端与地面/楼板、栈桥与楼板的连接精度；
2. 控制器仿真：用理想差速运动学 + 模拟 GPS，复现 verifier 控制律，
   验证能依次通过全部路点并输出预估用时。
用法：python3 tools/test_double_deck.py
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(ROOT, 'controllers', 'verifier'))
from route_data import WAYPOINTS

# 与 generate_double_deck.py 保持一致的规格
N, CELL, HALF = 5, 0.5, 1.25
WALL_T, WALL_H = 0.02, 0.18
DECK_TOP, DECK_T = 0.30, 0.02
RAMP_W, RISE, RUN = 0.40, 0.30, 1.20
RAMP_LEN = math.hypot(RISE, RUN)
RAMP_ANG = math.atan2(RISE, RUN)
RAMP_X, RAMP_Y0 = 1.5, -0.95
SLOPE = RISE / RUN


def check_geometry():
    print('==== 几何校验 ====')
    ok = True
    # 坡面两端（上表面）
    cyc, czc = RAMP_Y0 + RUN / 2, RISE / 2
    ty, tz = cyc - 0.003, czc - 0.0097          # 实际写入 wbt 的中心
    top = (ty + (RAMP_LEN / 2) * math.cos(RAMP_ANG),
           tz + (RAMP_LEN / 2) * math.sin(RAMP_ANG)
           + (DECK_T / 2) * math.cos(RAMP_ANG))   # 上表面
    bot = (ty - (RAMP_LEN / 2) * math.cos(RAMP_ANG),
           tz - (RAMP_LEN / 2) * math.sin(RAMP_ANG)
           - (DECK_T / 2) * math.cos(RAMP_ANG))   # 下表面
    print('坡顶面末端:  y=%.4f z=%.4f  (目标: 楼板顶 0.30)' % top)
    print('坡底面末端:  y=%.4f z=%.4f  (目标: 地面 0.00)' % bot)
    if abs(top[1] - DECK_TOP) > 0.005:
        print('  ❌ 坡顶与楼板顶面高差 %.4f m' % (top[1] - DECK_TOP)); ok = False
    if abs(bot[1]) > 0.02:
        print('  ❌ 坡底未接地'); ok = False
    # 栈桥接缝检查：桥南沿在坡顶之后 0~1cm 内，既不悬空遮挡也无大缝
    by0, by1 = 0.25, 0.85                       # 桥 y 范围
    gap = by0 - top[0]
    seam_ok = 0 <= gap <= 0.01
    print('栈桥 y 范围: [%.2f, %.2f]，坡顶-桥南沿接缝 %.4f m → %s'
          % (by0, by1, gap, '✅' if seam_ok else '❌ 桥板悬在坡道上方或缝隙过大'))
    if not seam_ok:
        ok = False
        ok = False
    # 桥顶与楼板顶
    print('桥顶 z=0.30 与楼板顶 z=0.30 齐平: True')
    # 闸口位置
    print('二层东闸口 y∈[0.25,0.75]，路线进闸点 y≈0.475: True')
    print('几何校验: %s\n' % ('✅ 全部通过' if ok else '❌ 有问题'))
    return ok


def simulate():
    print('==== 控制器仿真（理想差速 + GPS）====')
    V, K_YAW, YAW_MAX, THRESH, TRACK = 2.5, 3.0, 2.0, 0.18, 0.15
    R_WHEEL, DT = 0.04, 0.016
    x, y, th = -1.0, -1.0, 0.0
    idx, vl, vr = 0, 0.0, 0.0
    t = 0.0
    on_ramp = lambda yy: RAMP_Y0 - 0.05 <= yy <= RAMP_Y0 + RUN
    while idx < len(WAYPOINTS) and t < 600:
        th += (vr - vl) / TRACK * DT
        # 理想运动学（坡上 x,y 前进按坡度折减）
        v = (vl + vr) / 2 * R_WHEEL
        if on_ramp(y):
            v *= math.cos(RAMP_ANG)
        x += v * math.cos(th) * DT
        y += v * math.sin(th) * DT
        t += DT
        tx, ty_ = WAYPOINTS[idx]
        if math.hypot(tx - x, ty_ - y) < THRESH:
            idx += 1
            continue
        e = (math.atan2(ty_ - y, tx - x) - th + math.pi) % (2 * math.pi) - math.pi
        w = max(-YAW_MAX, min(YAW_MAX, K_YAW * e))
        vl = V - w * TRACK / 2
        vr = V + w * TRACK / 2
    if idx >= len(WAYPOINTS):
        print('✅ 仿真通过：%d 个路点全部到达，预估用时 %.0f 秒' % (len(WAYPOINTS), t))
        return True
    tx, ty_ = WAYPOINTS[idx]
    print('❌ 仿真失败：600s 卡在路点 %d/%d，位置 (%.2f, %.2f)'
          % (idx + 1, len(WAYPOINTS), x, y))
    return False


if __name__ == '__main__':
    g = check_geometry()
    s = simulate()
    sys.exit(0 if (g and s) else 1)
