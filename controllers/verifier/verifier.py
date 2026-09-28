# -*- coding: utf-8 -*-
"""
验证机器人 v4：GPS 航向自校正 + 卡死脱困
==========================================
路线由 tools/generate_double_deck.py 预生成（route_data.py）。

控制策略：
  - 转向：偏航角速度控制 w = clamp(K_YAW·e, ±YAW_MAX)（温和，不打滑失控）
  - 航向 est：轮速差积分 + 移动时 GPS 位移校正（滑移导致的漂移 0.25s 内消除）
  - 脱困：3 秒没到下一个路点且 1 秒内位移 <3cm → 倒车 0.8s → 原地转 90° → 继续
历史教训：
  v2 全速差原地转（2000°/s）打滑失控；v3 里程计无 GPS 校正，
  滑移转向误差累积 → 起步就斜开撞墙 → 无脱困逻辑卡死。
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
K_YAW = 3.0     # 航向比例增益
YAW_MAX = 2.0   # 最大偏航角速度 rad/s
THRESH = 0.18   # 路点到达半径 m
TRACK = 0.14    # 轮距 m
TIMEOUT = 600.0
GPS_DT = 0.25   # GPS 航向校正周期 s
STUCK_T = 3.0   # 多少秒没进步判定卡死
STUCK_MOVE = 0.03   # 1s 内位移小于此值 = 卡死


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
pos_hist = []        # (t, x, y) 近 1 秒位置
recover = None       # None / ('back', t_end) / ('turn', t_end)
turn_dir = 1
t_end = None

log('验证机器人启动：共 %d 个路点' % len(WAYPOINTS))

while robot.step(timestep) != -1:
    now = robot.getTime()

    # ---- 里程计航向积分 ----
    est += (vr_cmd - vl_cmd) / TRACK * dt

    p = gps.getValues()
    x, y = p[0], p[1]

    # ---- GPS 航向校正（仅在正常前进且有位移时）----
    if recover is None and last_gps is not None and now - last_gps_t >= GPS_DT:
        dx, dy = x - last_gps[0], y - last_gps[1]
        if math.hypot(dx, dy) > 0.02:
            est = math.atan2(dy, dx)
    if last_gps is None or now - last_gps_t >= GPS_DT:
        last_gps, last_gps_t = (x, y), now

    # ---- 到达终点 ----
    if idx >= len(WAYPOINTS):
        for w in left + right:
            w.setVelocity(0.0)
        log('✅ 验证通过：到达二层终点！全程用时 %.1f 秒' % (now - t0))
        break
    if now - t0 > TIMEOUT:
        for w in left + right:
            w.setVelocity(0.0)
        log('❌ 超时：卡在路点 %d/%d (%.2f, %.2f)' % (idx + 1, len(WAYPOINTS), x, y))
        break

    # ---- 路点推进 ----
    tx, ty = WAYPOINTS[idx]
    if math.hypot(tx - x, ty - y) < THRESH:
        log('  路点 %d/%d ✓  (%.2f, %.2f)' % (idx + 1, len(WAYPOINTS), tx, ty))
        idx += 1
        last_wp_t = now
        continue

    # ---- 卡死检测与脱困 ----
    pos_hist = [(t, px, py) for (t, px, py) in pos_hist if now - t <= 1.0]
    pos_hist.append((now, x, y))
    if recover is None and now - last_wp_t > STUCK_T and len(pos_hist) >= 3:
        moved = max(math.hypot(px - x, py - y) for (_, px, py) in pos_hist)
        if moved < STUCK_MOVE:
            recover = ('back', now + 0.8)
            log('  ⚠ 卡住（%.1fs 无进展），倒车脱困' % (now - last_wp_t))

    if recover is not None:
        phase, t_end = recover
        if phase == 'back':
            vl_cmd = vr_cmd = -V
            if now >= t_end:
                recover = ('turn', now + 1.05)
        else:  # turn ~90°
            vl_cmd, vr_cmd = (-1.2, 1.2) if turn_dir > 0 else (1.2, -1.2)
            if now >= t_end:
                recover = None
                turn_dir *= -1
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
