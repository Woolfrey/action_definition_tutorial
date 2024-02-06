Make sure to source ROS2 before beginning:
```
source /opt/ros/<version>/setup.bash
```
where `<version>` is the ROS2 version (e.g. Foxy, Humble, etc.). Then create package:
```
ros2 pkg create action_definition_tutorial
```
Navigate to directory:
```
cd action_definition_tutorial
```
Create action folder
```
mkdir action && cd action
```
Create action file:
```
Fibonacci.action
```
Edit the action:
```
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
```
Edit `CMakeLists.txt` file:
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
                           "action/Fibonacci.action")
```
Edit `package.xml`:
```
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```
Navigate to the root (in my case `cd ~/workspace/colcon`) and build:
```
colcon build --packages-select action_definition_tutorial
```
Check:
```
ros2 interface show action_definition_tutorial/action/Fibonacci
```

