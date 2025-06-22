#!/usr/bin/env python3
# encoding: utf-8
# Test script for new myproject features

import rospy
import time
import json
from std_msgs.msg import String
from std_srvs.srv import Empty

def test_init_pose_service():
    """Test the go_to_init_pose service"""
    print("Testing init pose service...")
    
    try:
        # Wait for service to be available
        rospy.wait_for_service('/robot/go_to_init_pose', timeout=5.0)
        go_to_init_pose = rospy.ServiceProxy('/robot/go_to_init_pose', Empty)
        
        # Call the service
        response = go_to_init_pose()
        print("✓ Init pose service called successfully")
        return True
        
    except rospy.ServiceException as e:
        print(f"✗ Service call failed: {e}")
        return False
    except rospy.ROSException as e:
        print(f"✗ Service not available: {e}")
        return False

def test_servo_control():
    """Test the low-level servo control"""
    print("Testing servo control...")
    
    try:
        # Create publisher for servo control
        servo_pub = rospy.Publisher('/robot/servo_control', String, queue_size=1)
        
        # Wait for publisher to be ready
        time.sleep(1)
        
        # Test 1: Move head to look left
        print("  Moving head left...")
        servo_pub.publish(json.dumps({"23": [200, 1000]}))  # head_pan to 200
        time.sleep(2)
        
        # Test 2: Move head to look right
        print("  Moving head right...")
        servo_pub.publish(json.dumps({"23": [800, 1000]}))  # head_pan to 800
        time.sleep(2)
        
        # Test 3: Move head back to center
        print("  Moving head to center...")
        servo_pub.publish(json.dumps({"23": [500, 1000]}))  # head_pan to center
        time.sleep(2)
        
        # Test 4: Move left arm
        print("  Moving left arm...")
        servo_pub.publish(json.dumps({
            "13": [800, 1000],  # left shoulder pitch
            "15": [500, 1000],  # left shoulder roll
            "17": [300, 1000]   # left elbow pitch
        }))
        time.sleep(3)
        
        # Test 5: Move back to safe position
        print("  Moving arm back to safe position...")
        servo_pub.publish(json.dumps({
            "13": [875, 1000],  # back to init positions
            "15": [500, 1000],
            "17": [500, 1000]
        }))
        time.sleep(2)
        
        print("✓ Servo control tests completed")
        return True
        
    except Exception as e:
        print(f"✗ Servo control test failed: {e}")
        return False

def test_action_status_monitoring():
    """Monitor action status messages"""
    print("Testing action status monitoring...")
    
    status_received = False
    
    def status_callback(msg):
        nonlocal status_received
        print(f"  Status: {msg.data}")
        status_received = True
    
    # Subscribe to action status
    status_sub = rospy.Subscriber('/robot/action_status', String, status_callback)
    
    # Wait for some status messages
    time.sleep(5)
    
    if status_received:
        print("✓ Action status monitoring working")
        return True
    else:
        print("✗ No action status messages received")
        return False

def main():
    """Main test function"""
    print("=== Testing myproject New Features ===")
    print()
    
    # Initialize ROS node
    rospy.init_node('test_new_features', anonymous=True)
    
    # Test results
    tests_passed = 0
    total_tests = 3
    
    # Test 1: Init pose service
    if test_init_pose_service():
        tests_passed += 1
    print()
    
    # Test 2: Servo control
    if test_servo_control():
        tests_passed += 1
    print()
    
    # Test 3: Action status monitoring
    if test_action_status_monitoring():
        tests_passed += 1
    print()
    
    # Summary
    print("=== Test Summary ===")
    print(f"Tests passed: {tests_passed}/{total_tests}")
    
    if tests_passed == total_tests:
        print("🎉 All tests passed!")
    else:
        print("⚠️  Some tests failed. Check the output above for details.")
    
    print()
    print("=== Usage Examples ===")
    print("To use these features manually:")
    print()
    print("1. Go to init pose:")
    print("   rosservice call /robot/go_to_init_pose")
    print()
    print("2. Control individual servos:")
    print("   rostopic pub /robot/servo_control std_msgs/String '{\"23\": [200, 1000]}'")
    print()
    print("3. Monitor action status:")
    print("   rostopic echo /robot/action_status")
    print()
    print("4. Execute pre-recorded actions:")
    print("   rostopic pub /robot/execute_action std_msgs/String \"wave\"")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Test interrupted")
    except KeyboardInterrupt:
        print("\nTest stopped by user") 