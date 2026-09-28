#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
单层迷宫验证资源生成器
======================
1. 用当前迷宫（seed=37, 7×7）求起点→终点 BFS 最短路径，写入验证控制器路点；
2. 复制 maze.wbt 生成 worlds/maze_verify.wbt：给占位机器人挂 GPS + IMU、
   控制器换成 maze_verifier。交付用的 maze.wbt 不做任何改动。

用法：python3 tools/make_maze_verifier.py
"""
import os
import sys
from collections import deque

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(ROOT, 'tools'))
import generate_maze as gm

N, CELL, HALF = 7, 0.5, 1.75
gm.N, gm.HALF = N, HALF
SEED = 37
DIRS = (('N', 0, 1), ('S', 0, -1), ('E', 1, 0), ('W', -1, 0))


def cc(i, j):
    return (-HALF + CELL / 2 + CELL * i, -HALF + CELL / 2 + CELL * j)


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


def main():
    walls, _ = gm.generate_maze(SEED)
    path = shortest_path(walls, (0, 0), (N - 1, N - 1))
    assert path and path[0] == (0, 0) and path[-1] == (N - 1, N - 1)
    print('主路径 %d 格' % len(path))

    dedup = []
    for i, j in path:
        w = cc(i, j)
        if not dedup or w != dedup[-1]:
            dedup.append(w)

    os.makedirs(os.path.join(ROOT, 'controllers', 'maze_verifier'), exist_ok=True)
    with open(os.path.join(ROOT, 'controllers', 'maze_verifier', 'route_data.py'),
              'w', encoding='utf-8') as f:
        f.write('# 自动生成：单层迷宫验证路线（tools/make_maze_verifier.py）\n'
                'WAYPOINTS = [\n' +
                ''.join('    (%.3f, %.3f),\n' % w for w in dedup) + ']\n')
    print('已写入 controllers/maze_verifier/route_data.py（%d 个路点）' % len(dedup))

    src = os.path.join(ROOT, 'worlds', 'maze.wbt')
    s = open(src, encoding='utf-8').read()
    assert 'controller "placeholder_car"' in s, 'maze.wbt 中找不到占位控制器'
    s = s.replace('controller "placeholder_car"', 'controller "maze_verifier"')
    old = '  children [\n    DEF CAR_BODY Shape {'
    new = ('  children [\n    GPS {\n      translation 0 0 0.06\n'
           '      name "gps"\n    }\n'
           '    InertialUnit {\n      translation 0 0 0.05\n'
           '      name "imu"\n    }\n'
           '    DEF CAR_BODY Shape {')
    assert old in s, 'maze.wbt 中找不到机器人 children 起点'
    s = s.replace(old, new, 1)
    dst = os.path.join(ROOT, 'worlds', 'maze_verify.wbt')
    open(dst, 'w', encoding='utf-8').write(s)
    print('已写入 worlds/maze_verify.wbt')


if __name__ == '__main__':
    main()
