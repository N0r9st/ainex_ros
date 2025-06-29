#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/11
# @author:aiden
# 自动踢球
import math
import rospy
import signal
from std_msgs.msg import Float64, String
from ainex_sdk import pid, misc, common
from ainex_example.color_common import Common
from ainex_example.pid_track import PIDTrack
from ainex_example.approach_object import ApproachObject
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect, ColorsDetect

class KickBallNode(Common):
    # 左右踢球的动作名
    left_shot_action_name = 'left_shot'
    right_shot_action_name = 'right_shot'
    
    # 图像处理大小  
    image_process_size = [160, 120]

    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.running = True
        # 存储检测结果
        self.objects_info = []
        self.count_miss = 0
        # 找球相关标志位
        self.start_index = 0
        self.start_find_ball = False
        # 初始化头部位置
        self.head_pan_init = 500  # 左右舵机的初始值
        self.head_tilt_init = 300 # 上下舵机的初始值
        self.head_time_stamp = rospy.get_time()
        # 初始化父类 
        super().__init__(name, self.head_pan_init, self.head_tilt_init)
        self.calib_config = common.get_yaml_data('/home/ubuntu/ros_ws/src/ainex_example/config/calib.yaml')
        # 存放检测结果
        self.rl_dis = None
        self.ud_dis = None
        # 初始化接近控制
        self.approach_object = ApproachObject(self.gait_manager, step_mode=0)
        self.approach_object.update_gait(dsp=[400, 0.2, 0.02])
        self.approach_object.update_stop_count(1)
        self.approach_object.update_gait_range(x_range=[-0.013, 0.013])
        self.approach_object.update_approach_stop_value(30, 0, 3)
        signal.signal(signal.SIGINT, self.shutdown)
        
        # 头部的pid追踪
        self.head_pan_range = [125, 875]  # 左右转动限制在这个范围， 125为右
        self.head_tilt_range = [260, 500]  # 上下限制在这个范围， 250为下
        self.pid_rl = pid.PID(0.1, 0.0, 0.001)
        self.pid_ud = pid.PID(0.1, 0.0, 0.001)
        self.head_pan_init = 500  # 左右舵机的初始值
        self.head_tilt_init = 300 # 上下舵机的初始值
        self.rl_track = PIDTrack(self.pid_rl, self.head_pan_range, self.head_pan_init)
        self.ud_track = PIDTrack(self.pid_ud, self.head_tilt_range, self.head_tilt_init)
        
        # 躯体的追踪参数
        self.yaw_stop = 40  # 躯体停止转动时头部左右舵机脉宽值和中位500的差值
        
        # 找球时头部经过的5个位置，左右舵机，上下舵机，时间ms
        # left_down, left_up, center_up, right_up, right_down
        self.find_ball_position = [[650, 300, 1000], 
                                   [650, 500, 1000], 
                                   [500, 500, 1000], 
                                   [350, 500, 1000],
                                   [350, 300, 1000]
                                   ]

        # 订阅颜色识别结果
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)  # 设置颜色
        # 初始化站姿  
        self.motion_manager.run_action('walk_ready')
        # 是否自动开始
        if rospy.get_param('~start', True):
            # 通知颜色识别准备，此时只显示摄像头原画
            target_color = rospy.get_param('~color', 'blue')  # 设置识别蓝色
            self.enter_func(None)
            self.set_color_srv_callback(String(target_color))  
            self.start_srv_callback(None)  # 开启颜色识别
            common.loginfo('start kick %s ball' % target_color)
    # 退出回调 
    def shutdown(self, signum, frame):
        with self.lock:
            self.running = False
            common.loginfo('%s shutdown' % self.name)
    # 设置颜色服务回调
    def set_color_srv_callback(self, msg):
        # 设置追踪颜色
        param = ColorDetect()
        param.color_name = msg.data
        param.detect_type = 'circle'
        param.use_name = True
        param.image_process_size = self.image_process_size
        param.min_area = 20
        param.max_area = self.image_process_size[0]*self.image_process_size[1]
        self.detect_pub.publish([param])
        
        common.loginfo('%s set_color' % self.name)
        return [True, 'set_color']

    def get_color_callback(self, msg):
        # 获取颜色识别结果
        self.objects_info = msg.data
        # 获取识别结果，根据结果计算距离
        for object_info in self.objects_info:
            if object_info.type == 'circle':
                object_info.x = object_info.x - self.calib_config['center_x_offset']
                self.rl_dis, self.ud_dis = self.head_track_process(object_info)
    # 头部PID跟踪
    def head_track_process(self, object_info):
        # 头部追踪
        if abs(object_info.x - object_info.width/2) < 10:
            object_info.x = object_info.width/2
        if abs(object_info.y - object_info.height/2) < 10:
            object_info.y = object_info.height/2
        rl_dis = self.rl_track.track(object_info.x, object_info.width/2)
        ud_dis = self.ud_track.track(object_info.y, object_info.height/2)
        self.motion_manager.set_servos_position(20, [[23, int(rl_dis)], [24, int(ud_dis)]])

        return rl_dis, ud_dis
    # 躯体追踪
    def body_track_process(self, rl_dis, ud_dis, ball_data):                                                
        # 左右根据头部左右舵机位置值进行调整
        if ball_data is not None:
            yaw_stop = 500 + math.copysign(self.yaw_stop, rl_dis - 500)
            yaw_stop_ = (yaw_stop - rl_dis)/9
            if 15 < abs(yaw_stop - rl_dis) < 27:
                yaw_stop_ = math.copysign(4, yaw_stop - rl_dis)
            if self.approach_object.process(500 - ud_dis, 0 + self.calib_config['center_x_offset'], yaw_stop_, self.head_tilt_range[0], 0, 0, ball_data.width, ball_data.height):           
                self.gait_manager.disable()
                if rl_dis > 500:
                    self.motion_manager.run_action(self.left_shot_action_name)  # 左脚踢
                else:
                    self.motion_manager.run_action(self.right_shot_action_name)  # 右脚踢
    # 找球过程 
    def find_ball_process(self):
        # 根据预设的扫描点进行找球
        if rospy.get_time() > self.head_time_stamp:
            if self.start_index > len(self.find_ball_position) - 1:
                self.start_index = 0
            rl_dis = self.find_ball_position[self.start_index][0]
            ud_dis = self.find_ball_position[self.start_index][1]
            self.rl_track.update_position(rl_dis)  # pid的输出值要跟着更新
            self.ud_track.update_position(ud_dis)
            self.motion_manager.set_servos_position(self.find_ball_position[self.start_index][2], [[23, rl_dis], [24, ud_dis]])

            self.gait_manager.move(2, 0, 0, 5)  # 右转
            self.head_time_stamp = rospy.get_time() + self.find_ball_position[self.start_index][2]/1000.0
            self.start_index += 1

    def run(self):
        while self.running:
            # 状态判断
            if self.start:
                ball_data = None 
                # 在检测到的对象中查找球
                for object_info in self.objects_info:
                    if object_info.type == 'circle':
                        ball_data = object_info
                # 如果识别到球就进行躯体追踪
                if self.rl_dis is not None:
                    self.body_track_process(self.rl_dis, self.ud_dis, ball_data)
                    self.rl_dis = None
                    self.count_miss = 0
                    self.start_find_ball = False
                # 如果未识别到球
                else:
                    # 如果未开始找球
                    if not self.start_find_ball:
                        self.count_miss += 1    # 未识别计数加1
                         # 超过阈值则开始找球
                        if self.count_miss > 100:
                            self.count_miss = 0
                            self.start_find_ball = True
                            self.start_index = 0
                        rospy.sleep(0.01)
                    # 如果已开始找球
                    else:
                        self.find_ball_process()   # 进行找球过程
                rospy.sleep(0.01)
            else:
                rospy.sleep(0.01)

        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    KickBallNode('kick_ball').run()
