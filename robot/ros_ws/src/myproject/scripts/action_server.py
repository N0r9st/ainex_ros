#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/09
# @author:aiden

import sys
import os
import time
import sqlite3 as sql
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse

# Import HiwonderServoController using the proper ainex_sdk import pattern
from ainex_sdk import hiwonder_servo_controller

class ActionServer:
    def __init__(self):
        rospy.init_node('action_server')
        
        # Action files directory
        self.action_dir = '/home/ubuntu/software/ainex_controller/action_bk'
        
        # Initialize the servo controller
        self.servo_controller = hiwonder_servo_controller.HiwonderServoController('/dev/ttyAMA0', 115200)
        
        # Subscribe to action commands
        rospy.Subscriber('/robot/execute_action', String, self.execute_action_callback, queue_size=1)
        
        # Service to get available actions
        rospy.Service('/robot/get_available_actions', Empty, self.get_available_actions_callback)
        
        # Publisher for action status
        self.action_status_pub = rospy.Publisher('/robot/action_status', String, queue_size=1)
        
        rospy.loginfo('Action server initialized with HiwonderServoController')
        
    def __del__(self):
        """Cleanup when the node is destroyed"""
        if hasattr(self, 'servo_controller'):
            self.servo_controller.close()
        
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