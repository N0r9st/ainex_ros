#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/09
# @author:aiden

import rospy
import sys
from std_msgs.msg import String

def action_status_callback(msg):
    """Callback to receive action status updates"""
    print(f"Action Status: {msg.data}")

def main():
    rospy.init_node('action_client')
    
    # Publisher to send action commands
    action_pub = rospy.Publisher('/robot/execute_action', String, queue_size=1)
    
    # Subscriber to receive action status
    rospy.Subscriber('/robot/action_status', String, action_status_callback, queue_size=1)
    
    # Wait for publisher to be ready
    rospy.sleep(1)
    
    if len(sys.argv) < 2:
        print("Usage: python3 action_client.py <action_name>")
        print("Example: python3 action_client.py go_forward_low")
        return
    
    action_name = sys.argv[1]
    print(f"Sending action command: {action_name}")
    
    # Publish the action command
    action_pub.publish(action_name)
    
    # Keep the node running to receive status updates
    rospy.spin()

if __name__ == '__main__':
    main() 