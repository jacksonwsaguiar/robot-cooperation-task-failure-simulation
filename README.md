# Multi-Robot Cooperative Task Simulation with Failure Recovery

This repository presents a ROS 2 framework for simulating cooperative tasks among multiple robots, with a specific focus on fault tolerance and dynamic task reallocation. The system, built upon ROS 2 Humble and the Gazebo simulator, implements a centralized coordinator node that manages a fleet of TurtleBot3 robots. The objective is to execute a multi-object pick-and-place task (simulated via "attaching" and "detaching"), with the ability to detect failures in individual robots and reassign their tasks to other operational agents in the system.

## Key Features

* **Multi-Robot Simulation:** A Gazebo environment featuring 3 TurtleBot3 robots, each operating within its own ROS namespace.
* **Sophisticated State Machine:** The control logic is managed by an explicit state machine that handles the complete task lifecycle (`IDLE`, `GOTO_PICKUP`, `ATTACHING`, `GOTO_DROPOFF`, `DETACHING`).
* **Robust Object Manipulation:** Utilizes the `gazebo_ros_link_attacher` plugin to reliably simulate the act of picking up and dropping off objects by creating and destroying physical joints in the simulation in real-time.
* **Heartbeat-Based Failure Detection:** Each agent periodically publishes a "heartbeat." A central node monitors these signals and declares a robot as "failed" if its heartbeat ceases for a configurable period.
* **Task Recovery and Reallocation:** When a robot fails mid-task, its assigned task is automatically released, returned to the queue of pending tasks, and reassigned to the first operational robot that becomes available.
* **Dynamic Task Assignment:** Tasks are assigned to available robots based on proximity, ensuring efficient distribution of work.

## System Architecture

The system is composed of three main components that communicate via the ROS 2 middleware.

1.  **Coordinator Node (`task_logic_node.py`):** The "brain" of the system. This centralized node is responsible for:
    * Maintaining the state of all robots (`OPERATIONAL`, `FAILED`) and their task statuses (`IDLE`, `GOTO_PICKUP`, etc.).
    * Monitoring the `/heartbeat` topic to detect failures.
    * Executing the `organize_tasks` function to assign pending tasks to available robots.
    * Managing the complete task lifecycle, from assignment to completion or recovery.

2.  **Robot Driver Module (`robot_driver.py`):** An abstraction class that serves as an "autopilot" for a single robot. Each instance:
    * Subscribes to its specific robot's odometry (`/odom`) and laser scan (`/scan`) topics.
    * Publishes velocity commands to the `/cmd_vel` topic.
    * Publishes its own liveness signal on the `/heartbeat` topic.
    * Provides a high-level interface (e.g., `get_velocity_to_goal`) for the coordinator node.

3.  **Simulation Environment (Gazebo):**
    * The `base.world` file defines the scene, the objects (cylinders), and the target area.
    * The `link_attacher` plugin is loaded into the world and provides the ROS 2 services (`/ATTACHLINK`, `/DETACHLINK`) that enable object manipulation.

## Installation and Dependencies

This project was developed and tested on **ROS 2 Humble with Ubuntu 22.04**.

**1. Prerequisites:**
* A functional ROS 2 Humble installation (the `ros-humble-desktop-full` variant is recommended).
* TurtleBot3 simulation packages: `sudo apt-get install ros-humble-turtlebot3*`
* ROS 2 development tools: `sudo apt-get install ros-dev-tools`

**2. Workspace Dependencies:**
The project requires the `gazebo_ros_link_attacher` plugin, which must be compiled from source.

**3. Installation Procedure:**
```bash
# Create and navigate to your workspace
mkdir -p ~/your_workspace/src
cd ~/your_workspace/src

# Clone this repository
# git clone <YOUR_GIT_REPOSITORY_URL> package_name
# Example:
git clone [https://github.com/your_user/robot-cooperation-task-failure-simulation.git](https://github.com/your_user/robot-cooperation-task-failure-simulation.git) multi_robot

# Return to the workspace root
cd ..

# Install all dependencies listed in the package.xml files
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Source the environment (add to your .bashrc to make it permanent)
echo "source ~/your_workspace/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
## Usage
The main launch file allows for starting the simulation with or without the control logic node.

To start only the simulation environment (Gazebo with robots):

```bash
ros2 launch multi_robot main.launch.py
```

To start the simulation world

```bash
ros2 launch multi_robot main.launch.py
```
To start the control node separately (if the simulation is already running):

```bash
# In a new terminal
ros2 run multi_robot task_logic_node
```
## Heartbeat function
Inspired by the 'heartbeat' mechanism, a common paradigm in software engineering for ensuring service availability, this robotic architecture implements an analogous concept to achieve fault tolerance by detecting when an agent becomes non-operational. Architectural Overview:
```bash
RobotDriver('tb3_0')           TaskLogicNode         RobotDriver('tb3_2')
(has a Publisher)           (has a Subscriptor)      (has a Publisher)
         |                        ^                        |
         |                        |                        |
         +---- msg("tb3_0") ----> | <---- msg("tb3_2") ----+
                                  |
                        PUBLIC TOPIC "/heartbeat"
                                  |
                                  ^
                                  |
                        +---- msg("tb3_1") ----+
                        |
                   RobotDriver('tb3_1')
                   (has a Publisher)
```

## Core Functionality: Failure and Recovery Mechanism
Fault tolerance is the core of this project. The flow of events is as follows:

1.  **Normal Operation:** Each `RobotDriver` publishes its name to the `/heartbeat` topic every second. The `TaskLogicNode` receives these signals and updates a timestamp for each robot.
2.  **Failure Simulation:** After a configured time (15 seconds), the `TaskLogicNode` commands one of the `RobotDriver`s to enter a failure state, ceasing its heartbeat publications.
3.  **Detection:** The `TaskLogicNode` periodically checks the timestamps. If the time since a robot's last heartbeat exceeds the `timeout` (3 seconds), that robot's status is changed from `OPERATIONAL` to `FAILED`.
4.  **Recovery:** The `organize_tasks` function detects that a `FAILED` robot still has an assigned task. It executes the following steps:
    * Sends a `/DETACHLINK` command to ensure any object the robot was carrying is released in the simulation.
    * Returns the original, complete task to the queue of pending tasks (`unassigned_tasks`).
    * Clears the failed robot's assignments.
5.  **Reallocation:** As soon as another robot completes its own task and becomes `IDLE`, the `organize_tasks` function sees an available agent and a pending task, and reassigns the abandoned task to the nearest functional robot.