# MyProject - Custom ROS Nodes for Ainex Robot

This package contains custom ROS nodes for the Ainex robot.

## Nodes

### Action Server (`action_server.py`)
A ROS node that acts as a server to accept action names from clients and execute them on the robot.

**Topics:**
- Subscribes to: `/robot/execute_action` (std_msgs/String)
- Publishes to: `/robot/action_status` (std_msgs/String)

**Services:**
- `/robot/get_available_actions` (std_srvs/Empty)

**Usage:**
```bash
# Launch the action server
roslaunch myproject action_server.launch

# Execute an action
rostopic pub -1 /robot/execute_action std_msgs/String "data: 'go_forward_low'"

# Monitor status
rostopic echo /robot/action_status
```

### Action Client (`action_client.py`)
A simple client to test the action server.

**Usage:**
```bash
python3 /path/to/myproject/scripts/action_client.py <action_name>
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