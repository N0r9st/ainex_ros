#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/04
# @author:aiden
# 跌倒起立
import cv2
import math
import rospy
import signal
import numpy as np
from ainex_sdk import common  # 导入通用功能函数
from threading import RLock  # 线程锁

from sensor_msgs.msg import Imu  # IMU消息类型
from std_srvs.srv import SetBool  # 设置布尔类型服务
from std_srvs.srv import Empty, EmptyResponse  # 空服务类型 

from ainex_interfaces.srv import SetFloat # 设置浮点数服务
from ainex_kinematics.gait_manager import GaitManager # 步态管理器
from ainex_kinematics.motion_manager import MotionManager # 动作管理器

class FallRiseNode:
    lie_to_stand_action_name = 'lie_to_stand'          # 向前倒起立动作名
    recline_to_stand_action_name = 'recline_to_stand'  # 向后倒起立动作名
    
    def __init__(self, name):
        self.name = name
        rospy.init_node(self.name)
        self.debug = False
        self.running = True
        self.point1 = [240, 230]
        self.point2 = [240, 30]
        
        # 前后倒计数
        self.count_lie = 0
        self.count_recline = 0
        
        self.state = 'stand'  # 当前状态
        self.start = False
        self.image = np.zeros((300, 480), np.uint8)
        self.imu_sub = None

        self.lock = RLock()
        
        # 初始化步态和动作管理器
        self.gait_manager = GaitManager()
        self.motion_manager = MotionManager()
        signal.signal(signal.SIGINT, self.shutdown)
        
        rospy.ServiceProxy('/sensor/imu/enable', SetBool)(True) # 启用IMU
        rospy.Service('~start', Empty, self.start_srv_callback) # 开始玩法
        rospy.Service('~stop', Empty, self.stop_srv_callback)   # 停止玩法

        rospy.sleep(0.2)
        rospy.set_param('~init_finish', True)
        self.motion_manager.run_action('stand')    # 播放站立动作
        common.loginfo('%s init_finish' % self.name)

        if rospy.get_param('~start', True):
            self.start_srv_callback(None)
            common.loginfo('start fall_rise')
    
    # 关闭处理函数
    def shutdown(self, signum, frame):
        self.running = False
        common.loginfo('%s shutdown' % self.name)
    
    # 开始服务回调
    def start_srv_callback(self, msg):
        with self.lock:
            if self.imu_sub is None:
                # 订阅imu话题
                self.imu_sub = rospy.Subscriber('/sensor/imu/imu_raw', Imu, self.imu_callback)
            self.start = True
        common.loginfo('%s start' % self.name)
        return EmptyResponse()
    
    # 停止服务回调
    def stop_srv_callback(self, msg):
        with self.lock:
            self.start = False
            if self.imu_sub is not None:
                self.imu_sub.unregister()  # 取消订阅
                self.imu_sub = None
        common.loginfo('%s stop' % self.name)
        return EmptyResponse()
    
    # 旋转变换点的函数
    def rotate(self, ps, m):
        pts = np.float32(ps).reshape([-1, 2])  # 要映射的点
        pts = np.hstack([pts, np.ones([len(pts), 1])]).T
        target_point = np.dot(m, pts).astype(np.int_)
        target_point = [[target_point[0][x], target_point[1][x]] for x in range(len(target_point[0]))]
        
        return target_point
    
    # 旋转一组点的函数
    def rotate_point(self, center_point, corners, angle):
        '''
        获取一组点绕一点旋转后的位置
        :param center_point:
        :param corners:
        :param angle:
        :return:
        '''
        # points [[x1, y1], [x2, y2]...]
        # 角度
        M = cv2.getRotationMatrix2D((center_point[0], center_point[1]), angle, 1)
        out_points = self.rotate(corners, M)

        return out_points
    
    # IMU回调函数 
    def imu_callback(self, msg):
        #转化为角度值
        angle = abs(int(math.degrees(math.atan2(msg.linear_acceleration.y, msg.linear_acceleration.z)))) 
        print(angle)
        # 根据状态计数
        if self.state == 'stand':
            if angle < 30:
                self.count_lie += 1
            else:
                self.count_lie = 0
            if angle > 150:
                self.count_recline += 1
            else:
                self.count_recline = 0
            # 根据计数判断是否进入起立状态
            if self.count_lie > 100:
                self.count_lie = 0
                self.state = 'lie_to_stand'
            elif self.count_recline > 100:
                self.count_recline = 0
                self.state = 'recline_to_stand'
            rospy.sleep(0.01)
        # 显示图像
        if self.debug:
            image = self.image.copy()
            # 通过图像显示角度
            point = self.rotate_point(self.point1, [self.point2], 90 - angle)[0]
            cv2.line(image, tuple(self.point1), tuple(point), (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(image, str(angle), (225, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.imshow('image', image)
            key = cv2.waitKey(1)

    # 主循环函数
    def run(self):
        while self.running:
            if self.start:  
                # 如果不是站立状态,进行起立
                if self.state != 'stand':
                    # 蜂鸣器提示
                    rospy.ServiceProxy('/sensor/buzzer/set_buzzer_frequency', SetFloat)(10)
                    rospy.sleep(1)
                    self.gait_manager.disable()  # 停止步态
                    # 根据状态选择起立动作
                    if self.state == 'lie_to_stand':
                        common.loginfo('lie_to_stand')
                        self.motion_manager.run_action(self.lie_to_stand_action_name)
                    elif self.state == 'recline_to_stand':
                        common.loginfo('recline_to_stand')
                        self.motion_manager.run_action(self.recline_to_stand_action_name)
                    rospy.sleep(0.5)
                    self.state = 'stand'  # 起立后切换到站立状态 
                else:
                    rospy.sleep(0.01)
            else:
                rospy.sleep(0.01)

        rospy.signal_shutdown('shutdown')

if __name__ == '__main__':
    FallRiseNode('fall_rise').run()
