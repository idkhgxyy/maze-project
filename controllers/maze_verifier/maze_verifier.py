# -*- coding: utf-8 -*-
"""
单层迷宫验证机器人 v2：IMU 航向 + 原地转向修正（bang-bang）
==========================================================
路线由 tools/make_maze_verifier.py 预生成（route_data.py）。

为什么这样控制（黑匣子日志的结论）：
  四轮固定底盘是滑移转向——温和的轮速差无法克服横向摩擦，
  车身几乎不旋转（v1 教训：一路直行撞墙）。
  因此：
  - 航向：IMU（惯性单元）直接输出 yaw，转多少度实测多少，不受打滑影响；
    开机自动校准：记录初始 raw 值（对应世界文件里已知的"朝东"），
    试转 0.4s 确定 raw→航向的符号映射，不依赖任何坐标系约定。
  - 修正：航向误差 >0.12 rad 就地原地对转修正（左右轮反转，four_wheel_car
    已验证该方式可精确转 90°），误差小则全速直行。
  - 定位：GPS 负责位置与到点判定。
"""
import math
import os
import sys

from controller import Robot

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from route_data import WAYPOINTS

LOG_PATH = os.path.join(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))), 'maze_verify_log.txt')


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

gps = robot.getDevice('gps')
gps.enable(timestep)
imu = robot.getDevice('imu')
imu.enable(timestep)

left = [robot.getDevice('wheel1'), robot.getDevice('wheel3')]
right = [robot.getDevice('wheel2'), robot.getDevice('wheel4')]
for w in left + right:
    w.setPosition(float('inf'))

V = 3.0         # 直行轮速 rad/s，线速度 0.12 m/s
P = 2.0         # 原地修正转向的轮速 rad/s（左右反转）
E_PIVOT = 0.12  # 航向误差超过 0.12 rad（约 7°）就原地修正
THRESH = 0.18   # 路点到达半径 m
STUCK_T = 3.0   # 卡死判定 s
STUCK_MOVE = 0.03
BACK_T = 0.7
TIMEOUT = 600.0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


idx = 1            # 路点 0 是出发点，直接跳过
vl_cmd = vr_cmd = 0.0
t0 = robot.getTime()
last_wp_t = t0
pos_hist = []
recover_t_end = None
next_diag = 0.0

# ---- IMU 校准状态机 ----
calib = 0          # 0=待记录 r0, 1=试转中, 2=完成
r0 = None
sign = 1
calib_t0 = None
tries = 0

log('验证启动：%d 个路点，起点 (-1.50, -1.50) → 终点 (1.50, 1.50)'
    % len(WAYPOINTS))

while robot.step(timestep) != -1:
    now = robot.getTime()
    p = gps.getValues()
    x, y, z = p[0], p[1], p[2]
    raw = imu.getValues()[2]

    if now >= next_diag:
        log('  [t=%5.1f] pos=(%.3f, %.3f, %.3f) raw=%7.3f est=%6.1f° '
            'cmd=(%5.2f,%5.2f) wp=%d/%d'
            % (now, x, y, z, raw, math.degrees(est) if calib == 2 else -999,
               vl_cmd, vr_cmd, idx + 1, len(WAYPOINTS)))
        next_diag = now + 1.0

    # ---- IMU 校准：试转 0.4s 确定 raw→航向 的符号 ----
    if calib == 0:
        r0 = raw
        calib = 1
        calib_t0 = now
        log('IMU 校准开始：r0=%.3f，原地试转 0.4s' % r0)
    if calib == 1:
        vl_cmd, vr_cmd = -V, V      # 左轮后退、右轮前进 = 逆时针
        if now - calib_t0 >= 0.4:
            delta = wrap(raw - r0)
            if abs(delta) > 0.2:
                sign = 1 if delta > 0 else -1
                calib = 2
                log('IMU 校准完成：Δraw=%.3f → sign=%+d' % (delta, sign))
            else:
                tries += 1
                if tries >= 3:
                    sign = 1
                    calib = 2
                    log('IMU 校准 3 次未检出旋转，默认 sign=+1')
                else:
                    calib_t0 = now
                    log('IMU 试转未检出（Δ=%.3f），重试 %d' % (delta, tries + 1))
        left[0].setVelocity(vl_cmd)
        left[1].setVelocity(vl_cmd)
        right[0].setVelocity(vr_cmd)
        right[1].setVelocity(vr_cmd)
        continue

    est = wrap(sign * wrap(raw - r0))

    # ---- 终点 / 超时 ----
    if idx >= len(WAYPOINTS):
        for w in left + right:
            w.setVelocity(0.0)
        log('✅ 验证通过：到达终点！全程用时 %.1f 秒' % (now - t0))
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

    # ---- 卡死检测 ----
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

    # ---- bang-bang 追踪：误差大就原地修正，误差小就直行 ----
    e = wrap(math.atan2(ty - y, tx - x) - est)
    if abs(e) > E_PIVOT:
        if e > 0:
            vl_cmd, vr_cmd = -P, P      # 需要左转：逆时针原地对转
        else:
            vl_cmd, vr_cmd = P, -P      # 需要右转：顺时针原地对转
    else:
        vl_cmd = vr_cmd = V

    left[0].setVelocity(vl_cmd)
    left[1].setVelocity(vl_cmd)
    right[0].setVelocity(vr_cmd)
    right[1].setVelocity(vr_cmd)
