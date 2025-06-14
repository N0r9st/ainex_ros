#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/03
# @author:aiden
# 视觉巡线节点
import rospy
import signal
from std_msgs.msg import String
from ainex_sdk import common
from ainex_example.color_common import Common
from ainex_example.visual_patrol import VisualPatrol
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect

class VisualPatrolNode(Common):
    # 按顺序检测三个roi，如果检测到黑线立刻跳出
    # y_min, y_max, x_min, x_max分别表示占图像的比例, 即实际大小为y_min*height
    line_roi = [(5 / 12, 6 / 12, 1 / 4, 3 / 4),
                (6 / 12, 7 / 12, 1 / 4, 3 / 4),
                (7 / 12, 8 / 12, 1 / 4, 3 / 4)
                ]
    # 图像处理时缩放到这个分辨率
    image_process_size = [160, 120]

    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.running = True
        self.objects_info = []
        # 初始化头部位置
        self.head_pan_init = 500   # 左右舵机的初始值
        self.head_tilt_init = 260  # 上下舵机的初始值
        super().__init__(name, self.head_pan_init, self.head_tilt_init)
        # 创建巡线控制实例
        self.visual_patrol = VisualPatrol(self.gait_manager)
        # 设置退出处理函数
        signal.signal(signal.SIGINT, self.shutdown)

        # 订阅颜色识别结果
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)
        # 初始化设置颜色服务
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)  
        self.motion_manager.run_action('walk_ready')     # 播放准备姿势动作
        # 如果自动开始参数为真,启动节点
        if rospy.get_param('~start', True):
            # 通知颜色识别准备，此时只显示摄像头原画
            target_color = rospy.get_param('~color', 'black')  # 设置识别黑色
            self.enter_func(None)
            self.set_color_srv_callback(String(target_color))  
            self.start_srv_callback(None)  # 开启颜色识别
            common.loginfo('start track %s lane' % target_color)
    # 节点关闭回调函数
    def shutdown(self, signum, frame):
        with self.lock:
            self.motion_manager.run_action('stand')
            self.running = False 
            common.loginfo('%s shutdown' % self.name)
            
    # 设置颜色服务回调函数
    def set_color_srv_callback(self, msg):
        # 生成颜色识别参数
        param = ColorDetect()
        param.color_name = msg.data
        param.use_name = True
        param.detect_type = 'line'  # 指定了检测类型为线条"line"
        param.image_process_size = self.image_process_size
        # 设置ROI参数
        param.line_roi.up.y_min = int(self.line_roi[0][0] * self.image_process_size[1])
        param.line_roi.up.y_max = int(self.line_roi[0][1] * self.image_process_size[1])
        param.line_roi.up.x_min = int(self.line_roi[0][2] * self.image_process_size[0])
        param.line_roi.up.x_max = int(self.line_roi[0][3] * self.image_process_size[0])

        param.line_roi.center.y_min = int(self.line_roi[1][0] * self.image_process_size[1])
        param.line_roi.center.y_max = int(self.line_roi[1][1] * self.image_process_size[1])
        param.line_roi.center.x_min = int(self.line_roi[1][2] * self.image_process_size[0])
        param.line_roi.center.x_max = int(self.line_roi[1][3] * self.image_process_size[0])

        param.line_roi.down.y_min = int(self.line_roi[2][0] * self.image_process_size[1])
        param.line_roi.down.y_max = int(self.line_roi[2][1] * self.image_process_size[1])
        param.line_roi.down.x_min = int(self.line_roi[2][2] * self.image_process_size[0])
        param.line_roi.down.x_max = int(self.line_roi[2][3] * self.image_process_size[0])
        # 面积过滤参数
        param.min_area = 1
        param.max_area = self.image_process_size[0] * self.image_process_size[1]
        # 发布颜色识别参数
        self.detect_pub.publish([param])
        common.loginfo('%s set_color' % self.name)
        return [True, 'set_color']
    
    # 发布颜色识别参数
    def get_color_callback(self, msg):
        # 获取颜色识别结果
        self.objects_info = msg.data

    # 主循环函数
    def run(self):
        while self.running:
            if self.start:
                # 获取识别结果
                line_data = None
                # stairs_data = None
                for object_info in self.objects_info:
                    if object_info.type == 'line':
                        line_data = object_info
                # 如果有识别结果,巡线控制
                if line_data is not None:
                    self.visual_patrol.process(line_data.x, line_data.width)
                rospy.sleep(0.01)
            else:
                rospy.sleep(0.01)
        # 退出前动作
        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    VisualPatrolNode('visual_patrol').run()
