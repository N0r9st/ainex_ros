#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/09
# @author:aiden

import sys
import os
import time
import sqlite3 as sql
import rospy
import yaml
import json
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist

# Import HiwonderServoController using the proper ainex_sdk import pattern
from ainex_sdk import hiwonder_servo_controller

class ActionServer:
    def __init__(self):
        rospy.init_node('action_server')
        
        # Action files directory
        self.action_dir = '/home/ubuntu/software/ainex_controller/action_bk'
        
        # Initialize the servo controller
        self.servo_controller = hiwonder_servo_controller.HiwonderServoController('/dev/ttyAMA0', 115200)
        
        # Load servo mapping and init pose
        self.load_servo_mapping()
        self.load_init_pose()
        
        # Subscribe to action commands
        rospy.Subscriber('/robot/execute_action', String, self.execute_action_callback, queue_size=1)
        
        # Subscribe to low-level servo control
        rospy.Subscriber('/robot/servo_control', String, self.servo_control_callback, queue_size=1)
        
        # Service to get available actions
        rospy.Service('/robot/get_available_actions', Empty, self.get_available_actions_callback)
        
        # Service to go to init pose
        rospy.Service('/robot/go_to_init_pose', Empty, self.go_to_init_pose_callback)
        
        # Publisher for action status
        self.action_status_pub = rospy.Publisher('/robot/action_status', String, queue_size=1)
        
        rospy.loginfo('Action server initialized with HiwonderServoController')
        
    def __del__(self):
        """Cleanup when the node is destroyed"""
        if hasattr(self, 'servo_controller'):
            self.servo_controller.close()
    
    def load_servo_mapping(self):
        """Load servo ID to joint name mapping"""
        self.servo_mapping = {
            # Left leg
            1: 'l_ank_roll',      # Left ankle roll
            3: 'l_ank_pitch',     # Left ankle pitch
            5: 'l_knee',          # Left knee
            7: 'l_hip_pitch',     # Left hip pitch
            9: 'l_hip_roll',      # Left hip roll
            11: 'l_hip_yaw',      # Left hip yaw
            
            # Right leg
            2: 'r_ank_roll',      # Right ankle roll
            4: 'r_ank_pitch',     # Right ankle pitch
            6: 'r_knee',          # Right knee
            8: 'r_hip_pitch',     # Right hip pitch
            10: 'r_hip_roll',     # Right hip roll
            12: 'r_hip_yaw',      # Right hip yaw
            
            # Left arm
            13: 'l_sho_pitch',    # Left shoulder pitch
            15: 'l_sho_roll',     # Left shoulder roll
            17: 'l_el_pitch',     # Left elbow pitch
            19: 'l_el_yaw',       # Left elbow yaw
            21: 'l_gripper',      # Left gripper
            
            # Right arm
            14: 'r_sho_pitch',    # Right shoulder pitch
            16: 'r_sho_roll',     # Right shoulder roll
            18: 'r_el_pitch',     # Right elbow pitch
            20: 'r_el_yaw',       # Right elbow yaw
            22: 'r_gripper',      # Right gripper
            
            # Head
            23: 'head_pan',       # Head pan
            24: 'head_tilt'       # Head tilt
        }
        
        # Reverse mapping for easy lookup
        self.joint_to_servo = {v: k for k, v in self.servo_mapping.items()}
        
    def load_init_pose(self):
        """Load initial pose configuration"""
        try:
            # Load from the ainex_kinematics config
            config_path = '/home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/config/init_pose.yaml'
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    init_pose_data = yaml.safe_load(f)
                    self.init_pose = init_pose_data.get('init_pose', {})
            else:
                # Fallback to default values
                self.init_pose = {
                    'l_ank_pitch': 0.625, 'l_ank_roll': 0.00, 'l_hip_pitch': -0.916,
                    'l_hip_roll': 0.00, 'l_hip_yaw': 0.00, 'l_knee': 1.192,
                    'l_sho_pitch': 0.00, 'l_sho_roll': 1.293, 'l_el_pitch': -0.10,
                    'l_el_yaw': -1.926, 'l_gripper': 0.00, 'r_ank_pitch': -0.625,
                    'r_ank_roll': 0.00, 'r_hip_pitch': 0.916, 'r_hip_roll': 0.00,
                    'r_hip_yaw': 0.00, 'r_knee': -1.192, 'r_sho_pitch': 0.00,
                    'r_sho_roll': -1.293, 'r_el_pitch': 0.10, 'r_el_yaw': 1.926,
                    'r_gripper': 0.00, 'head_pan': 0.00, 'head_tilt': 0.00
                }
            rospy.loginfo("Init pose loaded successfully")
        except Exception as e:
            rospy.logerr(f"Error loading init pose: {e}")
            self.init_pose = {}
        
    def get_available_actions(self):
        """Get list of available .d6a action files"""
        actions = []
        if os.path.exists(self.action_dir):
            for file in os.listdir(self.action_dir):
                if file.endswith('.d6a'):
                    actions.append(file)
        return actions
    
    def get_available_actions_callback(self, req):
        """Service callback to return available actions"""
        actions = self.get_available_actions()
        rospy.loginfo(f"Available actions: {actions}")
        return EmptyResponse()
    
    def go_to_init_pose_callback(self, req):
        """Service callback to move robot to initial pose"""
        rospy.loginfo("Moving robot to initial pose")
        self.action_status_pub.publish("STARTING: go_to_init_pose")
        
        try:
            self.move_to_init_pose()
            success_msg = "Robot moved to initial pose successfully"
            rospy.loginfo(success_msg)
            self.action_status_pub.publish("COMPLETED: go_to_init_pose")
        except Exception as e:
            error_msg = f"Error moving to init pose: {str(e)}"
            rospy.logerr(error_msg)
            self.action_status_pub.publish(f"ERROR: {error_msg}")
        
        return EmptyResponse()
    
    def move_to_init_pose(self):
        """Move robot to initial pose using servo positions"""
        # Convert joint angles to servo positions and move
        servo_positions = []
        
        for joint_name, angle in self.init_pose.items():
            if joint_name in self.joint_to_servo:
                servo_id = self.joint_to_servo[joint_name]
                # Convert angle to servo position (0-1000 range)
                # This is a simplified conversion - you may need to adjust based on your robot's kinematics
                servo_position = int(500 + (angle * 500))  # Center at 500, ±500 range
                servo_position = max(0, min(1000, servo_position))  # Clamp to valid range
                
                servo_positions.append([servo_id, servo_position])
                rospy.loginfo(f"Joint {joint_name} (servo {servo_id}): angle={angle}, position={servo_position}")
        
        # Move all servos to init pose
        if servo_positions:
            duration = 2000  # 2 seconds to reach init pose
            self.servo_controller.set_servos_position(duration, [servo_positions])
            time.sleep(duration/1000.0 + 0.1)
            rospy.loginfo("Init pose movement completed")
    
    def servo_control_callback(self, msg):
        """Callback for low-level servo control"""
        try:
            # Parse the servo control command
            # Expected format: JSON string like '{"1": [500, 1000], "23": [200, 500]}'
            # where keys are servo IDs (integers) and values are [position, time_ms]
            servo_data = []
            max_time = 1000  # default time
            
            servo_dict = json.loads(msg.data)
            
            for servo_id_str, value in servo_dict.items():
                servo_id = int(servo_id_str)
                position, time_ms = value
                
                # Validate servo ID
                if servo_id in self.servo_mapping:
                    servo_data.append([servo_id, int(position)])
                    max_time = max(max_time, int(time_ms))
                    rospy.loginfo(f"Servo {servo_id} ({self.servo_mapping[servo_id]}): position={position}, time={time_ms}ms")
                else:
                    rospy.logwarn(f"Invalid servo ID: {servo_id}")
            
            # Execute the servo movements
            if servo_data:
                self.servo_controller.set_servos_position(max_time, [servo_data])
                rospy.loginfo(f"Executed servo control with {len(servo_data)} servos")
            else:
                rospy.logwarn("No valid servo commands to execute")
                
        except Exception as e:
            error_msg = f"Error in servo control: {str(e)}"
            rospy.logerr(error_msg)
            self.action_status_pub.publish(f"ERROR: {error_msg}")
    
    def execute_action_callback(self, msg):
        """Callback to execute an action by name"""
        action_name = msg.data
        
        # Check if action file exists
        action_file = os.path.join(self.action_dir, action_name)
        if not action_name.endswith('.d6a'):
            action_file += '.d6a'
            
        if not os.path.exists(action_file):
            error_msg = f"Action file not found: {action_file}"
            rospy.logerr(error_msg)
            self.action_status_pub.publish(f"ERROR: {error_msg}")
            return
            
        rospy.loginfo(f"Executing action: {action_name}")
        self.action_status_pub.publish(f"STARTING: {action_name}")
        
        try:
            self.execute_d6a_action(action_file)
            success_msg = f"Action {action_name} completed successfully"
            rospy.loginfo(success_msg)
            self.action_status_pub.publish(f"COMPLETED: {action_name}")
        except Exception as e:
            error_msg = f"Error executing action {action_name}: {str(e)}"
            rospy.logerr(error_msg)
            self.action_status_pub.publish(f"ERROR: {error_msg}")
    
    def execute_d6a_action(self, action_file):
        """Execute a .d6a action file using HiwonderServoController"""
        rospy.loginfo(f"Loading action file: {action_file}")
        
        # Connect to the action database
        ag = sql.connect(action_file)
        cu = ag.cursor()
        cu.execute("select * from ActionGroup")
        
        # Execute the action sequence
        for step_num in range(10):  # Limit to 10 steps for safety
            act = cu.fetchone()
            if act is not None:
                rospy.loginfo(f"Executing step {step_num + 1}")
                
                # Prepare servo data for this step
                servo_data = []
                for i in range(0, len(act)-2, 1):
                    servo_id = i + 1
                    servo_position = act[2 + i]
                    servo_time = act[1]
                    
                    rospy.loginfo(f"Servo {servo_id}: position={servo_position}, time={servo_time}ms")
                    servo_data.append([servo_id, servo_position])
                
                # Use set_servos_position to move all servos at once
                if servo_data:
                    self.servo_controller.set_servos_position(servo_time, [servo_data])
                
                # Wait for the step to complete
                time.sleep(float(servo_time)/1000.0)
                rospy.loginfo(f"Step {step_num + 1} completed")
            else:
                rospy.loginfo("Action sequence completed")
                break
        
        # Close the database connection
        ag.close()
        rospy.loginfo("Action execution finished")

def main():
    try:
        action_server = ActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action server shutdown")
    finally:
        # Ensure servo controller is closed
        if 'action_server' in locals():
            action_server.servo_controller.close()

if __name__ == '__main__':
    main() 