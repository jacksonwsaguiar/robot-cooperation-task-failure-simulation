import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from .robot_driver import RobotDriver
import math
from linkattacher_msgs.srv import AttachLink, DetachLink

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

        # Criação dos drivers e da monitorização de Heartbeat
        self.drivers = { f'tb3_{i}': RobotDriver(self, f'tb3_{i}') for i in range(3) }
        self.heartbeat_subscriber = self.create_subscription(String, '/heartbeat', self.heartbeat_callback, 10)
        self.last_heartbeat_time = {name: self.get_clock().now() for name in self.drivers.keys()}
        self.robot_status = {name: 'OPERATIONAL' for name in self.drivers.keys()}
        self.status_check_timer = self.create_timer(1.0, self.check_robot_status)
        self.timeout = Duration(seconds=heartbeat_timeout_sec)
        
        # Clientes para os serviços de Attach/Detach
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        while not self.attach_client.wait_for_service(timeout_sec=1.0) or not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviços de attach/detach não disponíveis, a aguardar...')
        self.get_logger().info('Serviços de Attach/Detach encontrados.')
        
        # Gestão de Tarefas com a nova estrutura de "Objetos de Tarefa"
        self.tasks_list = [
            {'id': 0, 'waypoints': [(-3.0, 1.0), (-6.0, 1.0)]},
            {'id': 1, 'waypoints': [(-3.0, -1.0), (-6.0, -1.0)]},
            {'id': 2, 'waypoints': [(-3.0, -3.0), (-6.0, -3.0)]}
        ]
        self.unassigned_tasks = list(self.tasks_list)
        self.robot_assignments = {name: None for name in self.drivers.keys()}
        self.robot_current_waypoints = {name: [] for name in self.drivers.keys()}
        self.robot_task_status = {name: 'IDLE' for name in self.drivers.keys()}
        
        # Variáveis de controlo
        self.failure_triggered = False
        self.start_time = self.get_clock().now()
        # Temporizador principal da lógica
        self.timer = self.create_timer(0.1, self.logic_callback)
        self.get_logger().info('Nó de lógica da tarefa iniciado.')

    def attach_block(self, robot_name, block_id):
        """Chama o serviço para "colar" um bloco a um robô."""
        req = AttachLink.Request()
        req.model_name_1 = robot_name
        req.link_name_1 = 'base_link'
        req.model_name_2 = f"cilindro_{block_id}"
        req.link_name_2 = 'link'
        self.attach_client.call_async(req)
        self.get_logger().info(f"A pedir para 'colar' '{req.model_name_2}' em '{req.model_name_1}'")

    def detach_block(self, robot_name, block_id):
        """Chama o serviço para "descolar" um bloco de um robô."""
        req = DetachLink.Request()
        req.model_name_1 = robot_name
        req.link_name_1 = 'base_link'
        req.model_name_2 = f"cilindro_{block_id}"
        req.link_name_2 = 'link'
        self.detach_client.call_async(req)
        self.get_logger().info(f"A pedir para 'descolar' '{req.model_name_2}' de '{req.model_name_1}'")

    def organize_tasks(self):
        """Atribui tarefas pendentes aos robôs disponíveis e operacionais."""
        for robot_name, original_task in self.robot_assignments.items():
            if original_task is not None and self.robot_status[robot_name] == 'FAILED':
                self.get_logger().warn(f"Robô '{robot_name}' falhou! A tarefa {original_task['id']} está agora pendente.")
                self.unassigned_tasks.append(original_task)
                self.robot_assignments[robot_name] = None
                self.robot_current_waypoints[robot_name] = []
                self.robot_task_status[robot_name] = 'OUT_OF_SERVICE'

        available_robots = [name for name, status in self.robot_task_status.items() 
                            if status == 'IDLE' and self.robot_status[name] == 'OPERATIONAL']
        if not available_robots or not self.unassigned_tasks:
            return

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
                self.get_logger().info(f"TAREFA ATRIBUÍDA: Robô '{robot_name}' irá para a tarefa com ID {closest_task['id']}.")

    def logic_callback(self):
        """Callback principal, responsável por organizar e executar a máquina de estados."""
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
                twist_cmd, arrived = driver.get_velocity_to_goal(
                    current_target, self.linear_speed, self.angular_speed, self.goal_tolerance)
                if twist_cmd is not None: driver.publisher.publish(twist_cmd)
                if arrived:
                    self.get_logger().info(f"--- {name} no local de pickup. A 'colar' bloco. ---")
                    driver.stop()
                    self.robot_task_status[name] = 'ATTACHING_BLOCK'
            
            elif status == 'ATTACHING_BLOCK':
                original_task = self.robot_assignments[name]
                if original_task:
                    block_id = original_task['id']
                    self.attach_block(name, block_id)
                    self.robot_task_status[name] = 'GOTO_DROPOFF'
            
            elif status == 'GOTO_DROPOFF':
                # Remove o waypoint de pickup da lista de execução, se ainda não foi removido
                if len(waypoints) > 1:
                    waypoints.pop(0)
                
                if not waypoints: continue
                current_target = waypoints[0] # Agora o alvo é o ponto de entrega
                twist_cmd, arrived = driver.get_velocity_to_goal(
                    current_target, self.linear_speed, self.angular_speed, self.goal_tolerance)
                if twist_cmd is not None: driver.publisher.publish(twist_cmd)
                if arrived:
                    self.get_logger().info(f"--- {name} chegou ao local de entrega. A 'descolar' bloco. ---")
                    driver.stop()
                    self.robot_task_status[name] = 'DETACHING_BLOCK'

            elif status == 'DETACHING_BLOCK':
                original_task = self.robot_assignments[name]
                if original_task:
                    block_id = original_task['id']
                    self.detach_block(name, block_id)
                    self.get_logger().info(f"--- SUCESSO: {name} completou a entrega da tarefa com ID {block_id}. ---")
                    self.robot_task_status[name] = 'IDLE'
                    self.robot_assignments[name] = None
                    self.robot_current_waypoints[name] = []

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
                    self.get_logger().warn(f"HEARTBEAT PERDIDO de '{name}'! Estado mudado para 'FAILED'.")
                    self.robot_status[name] = 'FAILED'
            else:
                if self.robot_status[name] == 'FAILED':
                     self.get_logger().info(f"Heartbeat de '{name}' restabelecido. Estado mudado para 'OPERATIONAL'.")
                self.robot_status[name] = 'OPERATIONAL'

def main(args=None):
    rclpy.init(args=args)
    task_node = TaskLogicNode()
    rclpy.spin(task_node)
    task_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()