#!/usr/bin/env python3
# encoding: utf-8
# Date:2023/07/20
import time
from ainex_kinematics.motion_manager import MotionManager

# 调用上位机生成的动作, 参数为动作组存储的路径
motion_manager = MotionManager('/home/ubuntu/software/ainex_controller/ActionGroups')

# 单个舵机运行
motion_manager.set_servos_position(500, [[23, 300]])
time.sleep(0.5) 

# 多个舵机运行
motion_manager.set_servos_position(500, [[23, 500], [24, 500]])
time.sleep(0.5)

# 执行动作组，会阻塞直到运行完，所以不用加延时等待
motion_manager.run_action('left_shot')
motion_manager.run_action('right_shot')
