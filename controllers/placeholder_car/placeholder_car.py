# -*- coding: utf-8 -*-
"""
占位控制器：机器人保持静止，等待机器人组在此基础上实现走迷宫算法。

 motors wheel1..wheel4 已按速度模式初始化（左轮 = 1/3，右轮 = 2/4），
 差速驱动：左右轮同速直行，异速转弯，反向倒车。
"""
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_wheels = [robot.getDevice('wheel1'), robot.getDevice('wheel3')]
right_wheels = [robot.getDevice('wheel2'), robot.getDevice('wheel4')]
for w in left_wheels + right_wheels:
    w.setPosition(float('inf'))   # 速度控制模式
    w.setVelocity(0.0)            # 初始静止

print('占位机器人已加载：当前静止。')
print('请在本控制器基础上实现走迷宫算法（建议：距离传感器避障 + 转向策略）。')

while robot.step(timestep) != -1:
    pass  # 保持静止；机器人组的算法从这里开始
