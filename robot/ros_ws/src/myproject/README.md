# MyProject - Custom ROS Nodes for Ainex Robot

This package contains custom ROS nodes for the Ainex robot.

## Nodes

### Action Server (`action_server.py`)
A ROS node that acts as a server to accept action names from clients and execute them on the robot.

**Topics:**
- Subscribes to: `/robot/execute_action` (std_msgs/String)
- Subscribes to: `/robot/servo_control` (std_msgs/String) - Low-level servo control
- Publishes to: `/robot/action_status` (std_msgs/String)

**Services:**
- `/robot/get_available_actions` (std_srvs/Empty)
- `/robot/go_to_init_pose` (std_srvs/Empty) - Move robot to initial pose

**Usage:**
```bash
# Launch the action server
roslaunch myproject action_server.launch

# Execute an action
rostopic pub -1 /robot/execute_action std_msgs/String "data: 'go_forward_low'"

# Monitor status
rostopic echo /robot/action_status

# Go to initial pose
rosservice call /robot/go_to_init_pose

# Control individual servos
rostopic pub /robot/servo_control std_msgs/String '{"23": [200, 1000]}'  # Move head left
```

### Action Client (`action_client.py`)
A simple client to test the action server.

**Usage:**
```bash
python3 /path/to/myproject/scripts/action_client.py <action_name>
```

### Test New Features (`test_new_features.py`)
A test script to demonstrate and validate the new functionality.

**Usage:**
```bash
python3 /path/to/myproject/scripts/test_new_features.py
```

## New Features

### 1. Init Pose Service
The robot can now move to its initial standing pose using a ROS service.

**Service:** `/robot/go_to_init_pose`
**Type:** `std_srvs/Empty`

**Usage:**
```bash
rosservice call /robot/go_to_init_pose
```

This service:
- Loads the initial pose configuration from the robot's config files
- Converts joint angles to servo positions
- Moves all servos to their initial positions simultaneously
- Provides status updates via the `/robot/action_status` topic

### 2. Low-Level Servo Control
Direct control of individual servos through a ROS topic.

**Topic:** `/robot/servo_control`
**Type:** `std_msgs/String`

**Format:** JSON string with servo IDs as keys and [position, time_ms] as values
```
'{"servo_id": [position, time_ms], "servo_id": [position, time_ms], ...}'
```

**Examples:**
```bash
# Move head to look left
rostopic pub /robot/servo_control std_msgs/String '{"23": [200, 1000]}'

# Move left arm
rostopic pub /robot/servo_control std_msgs/String '{"13": [800, 1000], "15": [500, 1000], "17": [300, 1000]}'

# Move multiple leg joints
rostopic pub /robot/servo_control std_msgs/String '{"1": [500, 1000], "3": [600, 1000], "5": [400, 1000]}'
```

**Parameters:**
- `servo_id`: Servo ID (integer, 1-24, see Servo Mapping below)
- `position`: Target position (0-1000, where 500 is center)
- `time_ms`: Movement duration in milliseconds (20-30000)

## Servo Mapping

The AiNex robot uses 24 Hiwonder servos controlled via a serial bus. Each servo has a unique ID (1-24) that corresponds to a specific joint on the robot.

### Leg Servos (IDs 1-12)

#### Left Leg
| Servo ID | Joint Name | Description | Position Range | Default Position |
|----------|------------|-------------|----------------|------------------|
| 1 | l_ank_roll | Left ankle roll | 0-1000 | 500 |
| 3 | l_ank_pitch | Left ankle pitch | 0-1000 | 500 |
| 5 | l_knee | Left knee | 0-1000 | 240 |
| 7 | l_hip_pitch | Left hip pitch | 0-1000 | 500 |
| 9 | l_hip_roll | Left hip roll | 0-1000 | 500 |
| 11 | l_hip_yaw | Left hip yaw | 0-1000 | 500 |

#### Right Leg
| Servo ID | Joint Name | Description | Position Range | Default Position |
|----------|------------|-------------|----------------|------------------|
| 2 | r_ank_roll | Right ankle roll | 0-1000 | 500 |
| 4 | r_ank_pitch | Right ankle pitch | 0-1000 | 500 |
| 6 | r_knee | Right knee | 0-1000 | 760 |
| 8 | r_hip_pitch | Right hip pitch | 0-1000 | 500 |
| 10 | r_hip_roll | Right hip roll | 0-1000 | 500 |
| 12 | r_hip_yaw | Right hip yaw | 0-1000 | 500 |

### Arm Servos (IDs 13-22)

#### Left Arm
| Servo ID | Joint Name | Description | Position Range | Default Position |
|----------|------------|-------------|----------------|------------------|
| 13 | l_sho_pitch | Left shoulder pitch | 1000-0 (inverted) | 875 |
| 15 | l_sho_roll | Left shoulder roll | 0-1000 | 500 |
| 17 | l_el_pitch | Left elbow pitch | 0-1000 | 500 |
| 19 | l_el_yaw | Left elbow yaw | 0-1000 | 500 |
| 21 | l_gripper | Left gripper | 0-1000 | 500 |

#### Right Arm
| Servo ID | Joint Name | Description | Position Range | Default Position |
|----------|------------|-------------|----------------|------------------|
| 14 | r_sho_pitch | Right shoulder pitch | 1000-0 (inverted) | 125 |
| 16 | r_sho_roll | Right shoulder roll | 0-1000 | 500 |
| 18 | r_el_pitch | Right elbow pitch | 0-1000 | 500 |
| 20 | r_el_yaw | Right elbow yaw | 0-1000 | 500 |
| 22 | r_gripper | Right gripper | 0-1000 | 500 |

### Head Servos (IDs 23-24)

| Servo ID | Joint Name | Description | Position Range | Default Position |
|----------|------------|-------------|----------------|------------------|
| 23 | head_pan | Head pan (left/right) | 0-1000 | 500 |
| 24 | head_tilt | Head tilt (up/down) | 0-1000 | 500 |

### Position Values

- **Range**: 0-1000 (where 500 is typically the center position)
- **Units**: Digital servo position units
- **Direction**: 
  - 0 = minimum position (usually fully extended or rotated)
  - 500 = center position (neutral pose)
  - 1000 = maximum position (usually fully contracted or rotated)

### Special Notes

#### Inverted Servos
Some servos have inverted ranges:
- **Left shoulder pitch (ID 13)**: Range is 1000-0 (inverted)
- **Right shoulder pitch (ID 14)**: Range is 1000-0 (inverted)

#### Safety Limits
The servo controller automatically applies safety limits to certain servos:
- **Servo 11 (l_hip_yaw)**: Limited to 385-570
- **Servo 12 (r_hip_yaw)**: Limited to 430-615
- **Servo 21 (l_gripper)**: Limited to 200-620
- **Servo 22 (r_gripper)**: Limited to 380-800

### Python Dictionary Mapping

```python
servo_mapping = {
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
```

## Installation

1. Build the workspace:
```bash
cd /home/ubuntu/ros_ws
catkin_make
source devel/setup.bash
```

2. Launch the action server:
```bash
roslaunch myproject action_server.launch
```

## Action Files

The action server looks for `.d6a` action files in `/home/ubuntu/software/ainex_controller/action_bk/`.

## Testing

Run the test script to validate all features:
```bash
python3 /path/to/myproject/scripts/test_new_features.py
```

This will test:
- Init pose service functionality
- Low-level servo control
- Action status monitoring

## Safety Notes

- Always test movements in a safe environment
- Use the init pose service to return to a safe position
- Be aware of servo position limits (0-1000)
- Some servos have inverted ranges (see Servo Mapping above)
- The servo controller applies automatic safety limits to certain servos

## Troubleshooting

### Common Issues

1. **Servo not responding**: Check if the servo ID is correct and the servo is powered
2. **Position out of range**: Ensure position values are between 0-1000
3. **Movement too fast**: Increase the time_ms parameter for smoother movement
4. **Inverted movement**: Some servos (13, 14) have inverted ranges
5. **JSON parsing error**: Ensure the servo control message is valid JSON format

### Safety Guidelines

- Always start with small movements when testing new positions
- Test movements in a safe environment
- Be aware of the robot's physical limits
- Use the init pose service to return to a safe position if needed 