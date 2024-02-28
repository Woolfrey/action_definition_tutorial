# ROS2 Tutorial 3.1 Defining an Action

This is Part 3 in a series of ROS2 Tutorials:
1. [Publishers & Subscribers](https://github.com/Woolfrey/tutorial_publisher_subscriber)
2. Services
     1. [Defining a Service](https://github.com/Woolfrey/tutorial_service_definition)
     2. [Creating a Service & Client](https://github.com/Woolfrey/tutorial_service_client)
4. Actions
     1. [Defining an Action](https://github.com/Woolfrey/tutorial_action_definition)
     2. [Creating an Action Server & Client](https://github.com/Woolfrey/tutorial_action_server)
        
## Contents
- [What Are They?](#what-are-they)
- [Defining an Action](#defining-an-action)

## What are They?

Actions are one of 3 communication protocols in ROS2:

| Sender | Receiver | Node Interaction | Periodicity |
|--------|----------|------------------|-------------|
| Publisher | Subscriber | Indirect | Continuous |
| Server | Client | Direct | By request |
| Action Server | Action Client | Direct | By request, with continuous updates. |

- In the `publisher` & `subscriber` paradigm, a single publisher streams information to a topic. Multiple subscribers may then connect to this topic and use this information as they please. A publisher is suitable for streaming information, like sensor data on a robot.

<img src="https://github.com/Woolfrey/tutorial_action_definition/assets/62581255/8fb65629-6dd1-46ac-9d85-2a4eb794a16d" alt="image" width="300" height="auto">

- A `server` processes a one-time request from a `client`. This is suited to information that is required sporadically. For example, retrieving an update of a map, or generating a new path to a desired location. The information is passed directly between nodes that others cannot access.
- The `action` protocol amalgmates the concept of the publisher & subscriber with that of the client & server. An `action client` makes a request to an `action server`. Whilst processing this request, the action server publishes information on its progress.

<img src="https://github.com/Woolfrey/tutorial_action_definition/assets/62581255/3f2a0cd3-7664-4d56-bcc4-d6f057790604" alt="image" width = "300" height="auto">

Actions are suitable for structured tasks, for example telling a robot to drive to a particular location. It is a finite task that is executed infrequently, hence the server/client feature. But we may want to receive continual updates on its progress (time until completion, tracking accuracy, etc.), hence the publisher/subscriber aspect.

<img src="https://github.com/Woolfrey/tutorial_action_definition/assets/62581255/6aad8ead-929f-4a87-8bb2-d521f0825d33" alt="image" width="300" height="auto">

An `Action.action` file is structured as follows:
```
# Goal
datatype goal
---
# Result
datatype result
---
# Feedback
datatype feedback
```

:arrow_backward: [Go back.](#ros2-tutorial-31-defining-an-action)

## Defining an Action

1. Make sure to source ROS2 before beginning:
```
source /opt/ros/<version>/setup.bash
```
where `<version>` is the ROS2 version (e.g. `Foxy`, `Humble`, etc.). Then create package:

2. In your ROS2 working directory, go to the `src` folder and create the following package:
```
ros2 pkg create tutorial_action_definition
```
3. Navigate in to this direction and create an `action` folder:
```
cd tutorial_action_definition
mkdir action && cd action
```
4. Create a file named `Haiku.action` inside the folder and edit its contents as follows:
Edit the action:
```
# Goal
int32 number_of_lines
---
# Result
string poem
---
# Feedback
int32 line_number
string current_line
```
5. Now go back to `/tutoral_action_definition` and edit the `CMakeLists.txt` with the following before the `ament_package()` line:
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
                           "action/Fibonacci.action")
```
6. Edit `package.xml` with the following:
```
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```
7. Navigate to the root (in my case `cd ~/<workspace>/colcon`) and build:
```
colcon build --packages-select tutorial_action_definition
```
8. Be sure to locally source so that ROS2 can find the newly generated package:
```
source ./install/setup.bash
```
9. Now you can check that the package was successfully compiled:
```
ros2 interface show tutorial_action_definition/action/Haiku
```
You should see something like:

<img src="https://github.com/Woolfrey/tutorial_action_definition/assets/62581255/0bd97acf-7ef7-4739-ab26-ac7378e496b7" alt="image" width="900" height="auto">

:arrow_backward: [Go back.](#ros2-tutorial-31-defining-an-action)
