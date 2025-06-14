#!/usr/bin/env python3
# encoding: utf-8
# Date:2023/07/10
import rospy
from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager

rospy.init_node('gait_control_demo')
# 调用上位机生成的动作, 参数为动作组存储的路径
motion_manager = MotionManager('/home/ubuntu/software/ainex_controller/ActionGroups')
# 步态控制库
gait_manager = GaitManager()
rospy.sleep(0.2)

# 更多参数调节的控制，各参数含义请参考ainex_kinematics/src/ainex_kinematics/gait_manager
gait_param = gait_manager.get_gait_param()  # 获取当前步态参数
gait_param['pelvis_offset'] = 5
gait_param['step_height'] = 0.02
gait_param['z_swap_amplitude'] = 0.006
gait_param['body_height'] = 0.025
dsp = [400, 0.2, 0.02]
gait_manager.set_step(dsp, 0.02, 0, 0, gait_param, arm_swap=30, step_num=0)
rospy.sleep(3)

# 如果需要切换到动作组需要先关掉步态控制
gait_manager.disable()
motion_manager.run_action('left_shot')

motion_manager.run_action('hand_open')
gait_manager.set_step(dsp, 0.02, 0, 0, gait_param, arm_swap=0, step_num=0)  # 要保持手臂动作，需要关闭摆臂动作
rospy.sleep(3)  
gait_manager.stop()

gait_manager.set_step(dsp, 0.02, 0, 0, gait_param, arm_swap=30, step_num=3)  # 控制行走步数
