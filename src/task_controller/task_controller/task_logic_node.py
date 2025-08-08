import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from .robot_driver import RobotDriver
import math
from linkattacher_msgs.srv import AttachLink, DetachLink
from functools import partial

class TaskLogicNode(Node):
    def __init__(self):
        super().__init__('task_logic_node')

        # Declaração e obtenção de parâmetros
        self.declare_parameter('robot_params.linear_speed', 0.15)
        self.declare_parameter('robot_params.angular_speed', 0.8)
        self.declare_parameter('robot_params.goal_tolerance', 0.15)
        self.declare_parameter('robot_params.heartbeat_timeout', 3.0)
        self.linear_speed = self.get_parameter('robot_params.linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('robot_params.angular_speed').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('robot_params.goal_tolerance').get_parameter_value().double_value
        heartbeat_timeout_sec = self.get_parameter('robot_params.heartbeat_timeout').get_parameter_value().double_value

        # Mapeamento de nomes e criação dos drivers
        num_robots = 3
        self.robot_names = [f'tb3_{i}' for i in range(num_robots)]
        self.gazebo_model_names = {f'tb3_{i}': f'turtlebot_{i}' for i in range(num_robots)}
        self.drivers = {name: RobotDriver(self, name) for name in self.robot_names}

        # Monitorização de Heartbeat
        self.heartbeat_subscriber = self.create_subscription(String, '/heartbeat', self.heartbeat_callback, 10)
        self.last_heartbeat_time = {name: self.get_clock().now() for name in self.robot_names}
        self.robot_status = {name: 'OPERATIONAL' for name in self.robot_names}
        self.status_check_timer = self.create_timer(1.0, self.check_robot_status)
        self.timeout = Duration(seconds=heartbeat_timeout_sec)
       
        # Clientes para os serviços de Attach/Detach
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        while not self.attach_client.wait_for_service(timeout_sec=1.0) or not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Attach/detach services not available, waiting...')
        self.get_logger().info('Attach/Detach services found.')
       
        # Gestão de Tarefas
        self.tasks_list = [
            {'id': 0, 'waypoints': [(-3.0, 1.0), (-6.0, 0.8)]},
            {'id': 1, 'waypoints': [(-3.0, -1.0), (-6.0, -1.0)]},
            {'id': 2, 'waypoints': [(-3.0, -3.0), (-6.0, -3.0)]}
        ]
        self.unassigned_tasks = list(self.tasks_list)
        self.robot_assignments = {name: None for name in self.robot_names}
        self.robot_current_waypoints = {name: [] for name in self.robot_names}
        self.robot_task_status = {name: 'IDLE' for name in self.robot_names}
       
        self.failure_triggered = False
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.logic_callback)
        self.get_logger().info('Task logic node started.')

    def attach_block(self, robot_name, block_id):
        req = AttachLink.Request()
        req.model1_name = self.gazebo_model_names[robot_name]
        req.link1_name = 'base_link'
        req.model2_name = f"cilindro_{block_id}"
        req.link2_name = 'link'
        future = self.attach_client.call_async(req)
        future.add_done_callback(partial(self.attach_response_callback, robot_name=robot_name))
        self.get_logger().info(f"Requesting to attach '{req.model2_name}' to '{req.model1_name}'")

    def attach_response_callback(self, future, robot_name):
        try:
            future.result()
            self.get_logger().info(f"SUCCESS attaching block for '{robot_name}'.")
            if self.robot_current_waypoints[robot_name]:
                self.robot_current_waypoints[robot_name].pop(0)
            self.robot_task_status[robot_name] = 'GOTO_DROPOFF'
        except Exception as e:
            self.get_logger().error(f"EXCEPTION/FAILURE in attach service call for '{robot_name}': {e}")
            self.robot_task_status[robot_name] = 'RECOVER_BLOCK'

    def detach_block(self, robot_name, block_id, callback_fn):
        req = DetachLink.Request()
        req.model1_name = self.gazebo_model_names[robot_name]
        req.link1_name = 'base_link'
        req.model2_name = f"cilindro_{block_id}"
        req.link2_name = 'link'
        future = self.detach_client.call_async(req)
        future.add_done_callback(callback_fn)
        self.get_logger().info(f"Requesting to detach '{req.model2_name}' from '{req.model1_name}'")

    def detach_response_callback(self, future, robot_name):
        try:
            future.result()
            self.get_logger().info(f"SUCCESS detaching block for '{robot_name}'. Task complete.")
            self.robot_task_status[robot_name] = 'IDLE'
            self.robot_assignments[robot_name] = None
            self.robot_current_waypoints[robot_name] = []
        except Exception as e:
            self.get_logger().error(f"EXCEPTION/FAILURE in detach service call for '{robot_name}': {e}")
            self.robot_task_status[robot_name] = 'RECOVER_BLOCK'
           
    def recovery_detach_callback(self, future, original_task):
        """Callback usado APENAS para a recuperação de falhas."""
        try:
            future.result()
            self.get_logger().info(f"SUCCESS detaching block from failed robot. Task {original_task['id']} is now available.")
            # Só agora, após a confirmação, devolvemos a tarefa à fila
            self.unassigned_tasks.append(original_task)
        except Exception as e:
            self.get_logger().error(f"Could not detach block from failed robot: {e}. Task {original_task['id']} will be re-added anyway.")
            self.unassigned_tasks.append(original_task)

    def organize_tasks(self):
        for robot_name, original_task in self.robot_assignments.items():
            if original_task is not None and self.robot_status[robot_name] == 'FAILED' and self.robot_task_status[robot_name] != 'OUT_OF_SERVICE':
                self.get_logger().warn(f"Robot '{robot_name}' failed! Recovering its task {original_task['id']}.")
                block_id = original_task['id']
                # Chama o detach com o callback de recuperação
                self.detach_block(robot_name, block_id, partial(self.recovery_detach_callback, original_task=original_task))
               
                # Limpa o estado do robô falhado, mas NÃO devolve a tarefa ainda
                self.robot_assignments[robot_name] = None
                self.robot_current_waypoints[robot_name] = []
                self.robot_task_status[robot_name] = 'OUT_OF_SERVICE'

        available_robots = [name for name, status in self.robot_task_status.items() if status == 'IDLE' and self.robot_status[name] == 'OPERATIONAL']
        if not available_robots or not self.unassigned_tasks: return
        for robot_name in available_robots:
            if not self.unassigned_tasks: break
            driver = self.drivers[robot_name]
            if driver.current_pose is None: continue
            robot_pos = (driver.current_pose.position.x, driver.current_pose.position.y)
            closest_task = None
            min_distance = float('inf')
            for task in self.unassigned_tasks:
                pickup_point = task['waypoints'][0]
                distance = math.sqrt((pickup_point[0] - robot_pos[0])**2 + (pickup_point[1] - robot_pos[1])**2)
                if distance < min_distance:
                    min_distance = distance
                    closest_task = task
            if closest_task:
                self.robot_assignments[robot_name] = closest_task
                self.robot_current_waypoints[robot_name] = closest_task['waypoints'].copy()
                self.robot_task_status[robot_name] = 'GOTO_PICKUP'
                self.unassigned_tasks.remove(closest_task)
                self.get_logger().info(f"TASK ASSIGNED: Robot '{robot_name}' will go to task with ID {closest_task['id']}.")

    def logic_callback(self):
        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds / 1e9
        if not self.failure_triggered and elapsed_time > 15.0:
            self.drivers['tb3_1'].simulate_failure()
            self.failure_triggered = True

        self.organize_tasks()

        for name, driver in self.drivers.items():
            status = self.robot_task_status[name]
            waypoints = self.robot_current_waypoints[name]

            if status == 'GOTO_PICKUP':
                if not waypoints: continue
                current_target = waypoints[0]
                twist_cmd, arrived = driver.get_velocity_to_goal(current_target, self.linear_speed, self.angular_speed, self.goal_tolerance)
                if twist_cmd is not None: driver.publisher.publish(twist_cmd)
                if arrived:
                    self.get_logger().info(f"--- {name} at pickup location. Requesting to attach block. ---")
                    driver.stop()
                    self.robot_task_status[name] = 'ATTACHING_BLOCK'
           
            elif status == 'ATTACHING_BLOCK':
                original_task = self.robot_assignments[name]
                if original_task:
                    block_id = original_task['id']
                    self.attach_block(name, block_id)
                    self.robot_task_status[name] = 'WAITING_FOR_ATTACH'
           
            elif status == 'WAITING_FOR_ATTACH':
                pass

            elif status == 'GOTO_DROPOFF':
                if not waypoints:
                    self.get_logger().warn(f"Robot '{name}' in GOTO_DROPOFF state but has no waypoints. Recovering.")
                    self.robot_task_status[name] = 'RECOVER_BLOCK'
                    continue
                current_target = waypoints[0]
                twist_cmd, arrived = driver.get_velocity_to_goal(current_target, self.linear_speed, self.angular_speed, self.goal_tolerance)
                if twist_cmd is not None: driver.publisher.publish(twist_cmd)
                if arrived:
                    self.get_logger().info(f"--- {name} arrived at dropoff location. Requesting to detach block. ---")
                    driver.stop()
                    self.robot_task_status[name] = 'DETACHING_BLOCK'

            elif status == 'DETACHING_BLOCK':
                original_task = self.robot_assignments[name]
                if original_task:
                    block_id = original_task['id']
                    # Passa o callback de sucesso normal
                    self.detach_block(name, block_id, partial(self.detach_response_callback, robot_name=name))
                    self.robot_task_status[name] = 'WAITING_FOR_DETACH'

            elif status == 'WAITING_FOR_DETACH':
                pass
           
            elif status == 'RECOVER_BLOCK':
                abandoned_task = self.robot_assignments[name]
                if abandoned_task and abandoned_task not in self.unassigned_tasks:
                    self.unassigned_tasks.append(abandoned_task)
                self.robot_assignments[name] = None
                self.robot_current_waypoints[name] = []
                self.robot_task_status[name] = 'IDLE'

    def heartbeat_callback(self, msg):
        robot_name = msg.data
        if robot_name in self.last_heartbeat_time:
            self.last_heartbeat_time[robot_name] = self.get_clock().now()
           
    def check_robot_status(self):
        now = self.get_clock().now()
        for name, last_time in self.last_heartbeat_time.items():
            time_since_last_heartbeat = now - last_time
            if time_since_last_heartbeat > self.timeout:
                if self.robot_status[name] == 'OPERATIONAL':
                    self.get_logger().warn(f"HEARTBEAT LOST from '{name}'! Status changed to 'FAILED'.")
                    self.robot_status[name] = 'FAILED'
            else:
                if self.robot_status[name] == 'FAILED':
                     self.get_logger().info(f"Heartbeat from '{name}' re-established. Status changed to 'OPERATIONAL'.")
                self.robot_status[name] = 'OPERATIONAL'

def main(args=None):
    rclpy.init(args=args)
    task_node = TaskLogicNode()
    rclpy.spin(task_node)
    task_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()