# -*- coding: utf-8 -*-
"""
验证机器人 v5：航向完全由 GPS 实测位移决定（传感器方案）
========================================================
路线由 tools/generate_double_deck.py 预生成（route_data.py）。

控制策略：
  - 航向 est：每 0.1s 用 GPS 实测位移方向更新（移动超过 1.5cm 才更新）。
    不再使用轮速差推算——滑移转向的打滑会让推算航向严重失真（v3/v4 教训）。
  - 转向：w = clamp(K_YAW·e, ±YAW_MAX)，温和连续转向，不做原地急转。
  - 脱困：3s 无进展且 1s 内位移 <3cm → 倒车 0.7s → 继续（一动起来航向自动重校）。
"""
import math
import os
import sys

from controller import Robot

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from route_data import WAYPOINTS

LOG_PATH = os.path.join(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))), 'verifier_log.txt')


def log(msg):
    print(msg)
    try:
        with open(LOG_PATH, 'a', encoding='utf-8') as f:
            f.write(msg + '\n')
    except OSError:
        pass


try:
    open(LOG_PATH, 'w').close()
except OSError:
    pass

robot = Robot()
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0

gps = robot.getDevice('gps')
gps.enable(timestep)

left = [robot.getDevice('wheel1'), robot.getDevice('wheel3')]
right = [robot.getDevice('wheel2'), robot.getDevice('wheel4')]
for w in left + right:
    w.setPosition(float('inf'))

V = 2.5         # 轮速 rad/s，线速度 0.10 m/s
K_YAW = 2.5     # 航向比例增益
YAW_MAX = 1.5   # 最大偏航角速度 rad/s（约 86°/s）
THRESH = 0.18   # 路点到达半径 m
TRACK = 0.15    # 轮距 m
GPS_DT = 0.2    # GPS 航向更新周期 s（0.1m/s 下约 2cm 位移）
MOVE_MIN = 0.012    # 更新航向所需最小位移 m
STUCK_T = 3.0       # 卡死判定 s
STUCK_MOVE = 0.03   # 1s 内位移阈值 m
BACK_T = 0.7        # 倒车时长 s
TIMEOUT = 600.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


idx = 0
est = 0.0            # 初始朝东（+X）
vl_cmd = vr_cmd = 0.0
t0 = robot.getTime()
last_wp_t = t0
last_gps, last_gps_t = None, t0
pos_hist = []
recover_t_end = None

log('验证机器人启动：共 %d 个路点' % len(WAYPOINTS))

while robot.step(timestep) != -1:
    now = robot.getTime()
    p = gps.getValues()
    x, y = p[0], p[1]

    # ---- GPS 航向更新：看实际往哪移动（脱困倒车期间不更新）----
    if recover_t_end is None and last_gps is not None \
            and now - last_gps_t >= GPS_DT:
        dx, dy = x - last_gps[0], y - last_gps[1]
        if math.hypot(dx, dy) >= MOVE_MIN:
            est = math.atan2(dy, dx)
    if last_gps is None or now - last_gps_t >= GPS_DT:
        last_gps, last_gps_t = (x, y), now

    # ---- 终点 / 超时 ----
    if idx >= len(WAYPOINTS):
        for w in left + right:
            w.setVelocity(0.0)
        log('✅ 验证通过：到达二层终点！全程用时 %.1f 秒' % (now - t0))
        break
    if now - t0 > TIMEOUT:
        for w in left + right:
            w.setVelocity(0.0)
        log('❌ 超时：卡在路点 %d/%d (%.2f, %.2f)'
            % (idx + 1, len(WAYPOINTS), x, y))
        break

    # ---- 路点推进 ----
    tx, ty = WAYPOINTS[idx]
    if math.hypot(tx - x, ty - y) < THRESH:
        log('  路点 %d/%d ✓  (%.2f, %.2f)' % (idx + 1, len(WAYPOINTS), tx, ty))
        idx += 1
        last_wp_t = now
        continue

    # ---- 卡死检测：倒车后继续（一动航向就自动重校）----
    pos_hist = [(t, px, py) for (t, px, py) in pos_hist if now - t <= 1.0]
    pos_hist.append((now, x, y))
    if recover_t_end is None and now - last_wp_t > STUCK_T and len(pos_hist) >= 3:
        moved = max(math.hypot(px - x, py - y) for (_, px, py) in pos_hist)
        if moved < STUCK_MOVE:
            recover_t_end = now + BACK_T
            log('  ⚠ 卡住（%.1fs 无进展），倒车 %.1fs 脱困'
                % (now - last_wp_t, BACK_T))

    if recover_t_end is not None:
        vl_cmd = vr_cmd = -V
        if now >= recover_t_end:
            recover_t_end = None
            last_wp_t = now
        left[0].setVelocity(vl_cmd)
        left[1].setVelocity(vl_cmd)
        right[0].setVelocity(vr_cmd)
        right[1].setVelocity(vr_cmd)
        continue

    # ---- 正常追踪 ----
    e = wrap(math.atan2(ty - y, tx - x) - est)
    w = clamp(K_YAW * e, -YAW_MAX, YAW_MAX)
    vl_cmd = V - w * TRACK / 2
    vr_cmd = V + w * TRACK / 2

    left[0].setVelocity(vl_cmd)
    left[1].setVelocity(vl_cmd)
    right[0].setVelocity(vr_cmd)
    right[1].setVelocity(vr_cmd)
