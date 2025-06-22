#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/09
# @author:aiden

import sys
import time
import os
import sqlite3 as sql
import argparse

# Add the ainex_sdk path to import HiwonderServoController
from ainex_sdk.hiwonder_servo_controller import HiwonderServoController

def execute_action(servo_controller, action_name):
    """
    Execute a single action from .d6a file using HiwonderServoController
    """
    # Action files directory
    action_dir = '/home/ubuntu/software/ainex_controller/action_bk'
    action_file = os.path.join(action_dir, action_name)
    
    # Add .d6a extension if not present
    if not action_name.endswith('.d6a'):
        action_file += '.d6a'
    
    # Check if action file exists
    if not os.path.exists(action_file):
        print(f"ERROR: Action file not found: {action_file}")
        return False
    
    print(f"Executing action: {action_name}")
    
    try:
        # Connect to the action database
        ag = sql.connect(action_file)
        cu = ag.cursor()
        cu.execute("select * from ActionGroup")
        
        # Execute the action sequence
        for step_num in range(10):  # Limit to 10 steps for safety
            act = cu.fetchone()
            if act is not None:
                print(f"  Step {step_num + 1}")
                
                # Prepare servo data for this step
                servo_data = []
                for i in range(0, len(act)-2, 1):
                    servo_id = i + 1
                    servo_position = act[2 + i]
                    servo_time = act[1]
                    
                    print(f"    Servo {servo_id}: position={servo_position}, time={servo_time}ms")
                    servo_data.append([servo_id, servo_position])
                
                # Use set_servos_position to move all servos at once
                if servo_data:
                    servo_controller.set_servos_position(servo_time, [servo_data])
                
                # Wait for the step to complete
                time.sleep(float(servo_time)/1000.0)
                print(f"    Step {step_num + 1} completed")
            else:
                print("  Action sequence completed")
                break
        
        print(f"Action '{action_name}' executed successfully")
        return True
        
    except Exception as e:
        print(f"ERROR: Failed to execute action '{action_name}': {str(e)}")
        return False

def list_available_actions():
    """List all available .d6a action files"""
    action_dir = '/home/ubuntu/software/ainex_controller/action_bk'
    actions = []
    
    if os.path.exists(action_dir):
        for file in os.listdir(action_dir):
            if file.endswith('.d6a'):
                actions.append(file)
    
    return actions

def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(
        description='Execute robot actions sequentially using HiwonderServoController',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s go_forward_low
  %(prog)s go_forward_low turn_left walk_backward
  %(prog)s --list
  %(prog)s --delay 2.0 go_forward_low turn_left
        """
    )
    
    parser.add_argument('actions', nargs='*', help='Action names to execute (without .d6a extension)')
    parser.add_argument('--list', '-l', action='store_true', help='List all available actions')
    parser.add_argument('--delay', '-d', type=float, default=1.0, 
                       help='Delay between actions in seconds (default: 1.0)')
    parser.add_argument('--port', '-p', default='/dev/ttyAMA0',
                       help='Serial port for servo controller (default: /dev/ttyAMA0)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200,
                       help='Baudrate for serial communication (default: 115200)')
    
    args = parser.parse_args()
    
    # List available actions if requested
    if args.list:
        actions = list_available_actions()
        if actions:
            print("Available actions:")
            for action in sorted(actions):
                print(f"  {action}")
        else:
            print("No .d6a action files found in /home/ubuntu/software/ainex_controller/action_bk/")
        return
    
    # Check if actions were provided
    if not args.actions:
        parser.print_help()
        return
    
    # Initialize the servo controller
    try:
        print(f"Initializing servo controller on {args.port} at {args.baudrate} baud...")
        servo_controller = HiwonderServoController(args.port, args.baudrate)
        print("Servo controller initialized successfully")
    except Exception as e:
        print(f"ERROR: Failed to initialize servo controller: {str(e)}")
        return
    
    try:
        # Execute each action sequentially
        total_actions = len(args.actions)
        successful_actions = 0
        
        for i, action_name in enumerate(args.actions, 1):
            print(f"\n[{i}/{total_actions}] Processing action: {action_name}")
            
            if execute_action(servo_controller, action_name):
                successful_actions += 1
            else:
                print(f"Failed to execute action: {action_name}")
            
            # Add delay between actions (except after the last one)
            if i < total_actions and args.delay > 0:
                print(f"Waiting {args.delay} seconds before next action...")
                time.sleep(args.delay)
        
        # Summary
        print(f"\nExecution complete: {successful_actions}/{total_actions} actions successful")
        
    finally:
        # Clean up - servo controller will be closed automatically by destructor
        print("Execution finished")

if __name__ == '__main__':
    main() 