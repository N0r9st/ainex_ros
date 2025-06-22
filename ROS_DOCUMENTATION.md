# ROS System Documentation

This document provides comprehensive information about the ROS nodes, topics, and services in the Ainex project.

## Table of Contents
1. [ROS Nodes](#ros-nodes)
2. [ROS Topics](#ros-topics)
3. [ROS Services](#ros-services)
4. [Camera Stream](#camera-stream)
5. [Command Line Examples](#command-line-examples)

## ROS Nodes

The following nodes are currently running in the system:

### Core Nodes
- `/ainex_controller` - Main controller node
- `/app` - Application node
- `/rosout` - Standard ROS logging node
- `/rosbridge_websocket` - WebSocket bridge for ROS
- `/rosapi` - ROS API server

### Camera and Vision Nodes
- `/camera` - Main camera node
- `/camera/manager` - Camera nodelet manager
- `/camera/rectify_color` - Color image rectification node
- `/color_detection` - Color detection node
- `/face_detect` - Face detection node
- `/web_video_server` - Web video streaming server

### Sensor and Control Nodes
- `/imu_calib` - IMU calibration node
- `/imu_filter` - IMU data filtering node
- `/joystick` - Joystick input node
- `/joystick_control` - Joystick control node
- `/sensor` - Sensor management node

## ROS Topics

### Application Topics
- `/app/image_result` - Image processing results
- `/app/set_action` - Action commands
- `/app/set_walking_param` - Walking parameters

### Camera Topics
- `/camera/camera_info` - Camera calibration information
- `/camera/image_raw` - Raw camera images
- `/camera/image_rect_color` - Rectified color images
- Various compressed and theora versions of the above topics

### Control Topics
- `/head_pan_controller/command` - Head pan control
- `/head_tilt_controller/command` - Head tilt control
- `/walking/is_walking` - Walking state
- `/walking/set_param` - Walking parameters

### Sensor Topics
- `/imu` - IMU data
- `/imu_corrected` - Corrected IMU data
- `/joy` - Joystick input
- `/sensor/button/get_button_state` - Button states
- `/sensor/imu/imu_mag` - IMU magnetometer data
- `/sensor/imu/imu_raw` - Raw IMU data

### Vision Topics
- `/color_detection/image_result` - Color detection results
- `/color_detection/update_detect` - Color detection updates
- `/face_detect/image_result` - Face detection results
- `/object/pixel_coords` - Object pixel coordinates

### System Topics
- `/client_count` - Connected client count
- `/connected_clients` - Connected clients information
- `/diagnostics` - System diagnostics
- `/rosout` - ROS logging
- `/rosout_agg` - Aggregated ROS logging
- `/tf` - Transform tree

## ROS Services

### Application Services
- `/app/enter` - Enter application mode
- `/app/get_target_color` - Get target color
- `/app/heartbeat` - Application heartbeat
- `/app/set_running` - Set running state
- `/app/set_target_color` - Set target color
- `/app/set_threshold` - Set detection threshold

### Camera Services
- `/camera/set_camera_info` - Set camera calibration
- `/camera/start_capture` - Start camera capture
- `/camera/stop_capture` - Stop camera capture
- Various parameter setting services for image compression

### Vision Services
- `/color_detection/enter` - Enter color detection mode
- `/color_detection/exit` - Exit color detection mode
- `/color_detection/start` - Start color detection
- `/color_detection/stop` - Stop color detection
- `/color_detection/update_lab` - Update LAB color space
- `/face_detect/enter` - Enter face detection mode
- `/face_detect/exit` - Exit face detection mode
- `/face_detect/start` - Start face detection
- `/face_detect/stop` - Stop face detection

### Sensor Services
- `/sensor/button/enable` - Enable button
- `/sensor/buzzer/set_buzzer_frequency` - Set buzzer frequency
- `/sensor/buzzer/set_buzzer_state` - Set buzzer state
- `/sensor/imu/enable` - Enable IMU
- `/sensor/led/set_led_state` - Set LED state
- `/sensor/rgb/set_rgb_state` - Set RGB state

### Walking Services
- `/walking/command` - Walking commands
- `/walking/get_param` - Get walking parameters
- `/walking/init_pose` - Initialize walking pose
- `/walking/is_walking` - Check walking state

### System Services
- Various logger configuration services for all nodes
- ROS API services for system information and control

## Camera Stream

The camera stream can be accessed through the web video server at:
```
http://192.168.149.1:8080/stream?topic=/camera/image_rect_color
```

This URL provides a live stream of the rectified color images from the camera. You can also access the raw camera feed at:
```
http://192.168.149.1:8080/stream?topic=/camera/image_raw
```

## Command Line Examples

Below are examples of how to interact with various topics and services using the command line. 

**Note:** To use these commands for topics and services with custom message types (like those in `ainex_interfaces`), you must first source your workspace's setup file. For example: `source ~/ainex_project/ainex_on_pc/robot/ros_ws/devel/setup.bash`. You can find the exact structure of a message by running `rosmsg show <package_name>/<message_type>`.

### Publishing to Topics

#### Application Control
```bash
# Set action
rostopic pub /app/set_action std_msgs/String "stand" --once
```

The following actions are available:
- `stair_down`
- `hand_back`
- `move_right`
- `move_left`
- `left_hand_put_block`
- `put_down`
- `climb_stairs`
- `back_step`
- `recline_to_stand`
- `place_block`
- `go_forward_low`
- `hand_open`
- `turn_right`
- `wave`
- `clamp_left`
- `forward_one_step`
- `move_up`
- `turn_left`
- `clamp_right`
- `crawl_right`
- `walk_ready`
- `forward`
- `left_shot`
- `greet`
- `crawl_left`
- `stand_low`
- `stair_up`
- `lie_to_stand`
- `twist`
- `forward_step`
- `hurdles`
- `go_turn_left_low`
- `go_turn_right_low`
- `descend_stairs`
- `turn_left_30`
- `go_turn_left`
- `go_turn_right_20`
- `turn_right_30`
- `go_turn_right`
- `right_hand_put_block`
- `go_turn_left_20`
- `stand`
- `right_shot`
- `back`
- `raise_right_hand`
- `calib`

### Calling Services

#### Camera Control
```bash
# Start camera capture
rosservice call /camera/start_capture

# Stop camera capture
rosservice call /camera/stop_capture

# Set camera calibration
rosservice call /camera/set_camera_info "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
height: 480
width: 640
distortion_model: 'plumb_bob'
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]"
```

#### Vision Control
```bash
# Start color detection
rosservice call /color_detection/start

# Stop color detection
rosservice call /color_detection/stop

# Set target color (in LAB color space)
rosservice call /color_detection/update_lab "data: [50, 0, 0]"

# Start face detection
rosservice call /face_detect/start

# Stop face detection
rosservice call /face_detect/stop
```

#### Sensor Control
```bash
# Enable IMU
rosservice call /sensor/imu/enable "data: true"

# Set buzzer frequency (in Hz). Service Type: ainex_interfaces/SetBuzzer
rosservice call /sensor/buzzer/set_buzzer_frequency "frequency: 440"

# Set LED state. Service Type: ainex_interfaces/SetLED
rosservice call /sensor/led/set_led_state "{led_name: 'led1', state: true}"

# Set RGB state (R,G,B values 0-255). Service Type: ainex_interfaces/SetRGB
rosservice call /sensor/rgb/set_rgb_state "{red: 255, green: 0, blue: 0}"
```

#### Walking Control
```bash
# Command the robot to walk. Service Type: ainex_interfaces/SetWalkingCommand
rosservice call /walking/command "{command: 'forward', steps: 10}"

# Initialize walking pose
rosservice call /walking/init_pose

# Get walking parameters
rosservice call /walking/get_param
```

### Monitoring Topics

```bash
# Monitor camera images
rostopic echo /camera/image_rect_color

# Monitor IMU data
rostopic echo /imu_corrected

# Monitor color detection results
rostopic echo /color_detection/image_result

# Monitor face detection results
rostopic echo /face_detect/image_result
```

### Notes on Manual Publishing
- Only publish to control topics when you understand their message types and expected values
- Avoid publishing to internal system topics (like `/rosout`, `/tf`, etc.)
- Be careful when publishing to sensor topics as they might conflict with actual sensor data
- Some topics require specific message types and formats - use `rostopic info <topic_name>` to check the message type
- Use `--once` flag when testing to send a single message, or omit it for continuous publishing
- For services, use `rosservice call <service_name> <args>` format
- Always check the current state before sending commands to avoid conflicts 

## Notes
- All nodes have standard ROS logger services (`get_loggers`, `set_logger_level`)
- The exact format for custom messages and services should be confirmed using `rosmsg show` and `rossrv show`. 