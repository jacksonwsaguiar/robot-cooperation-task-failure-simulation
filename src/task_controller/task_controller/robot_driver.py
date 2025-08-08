import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion
import math

class RobotDriver:
    def __init__(self, node, robot_namespace):
        self.node = node
        self.namespace = robot_namespace
        self.is_failed = False

        cmd_vel_topic = f'/{self.namespace}/cmd_vel'
        odom_topic = f'/{self.namespace}/odom'
        scan_topic = f'/{self.namespace}/scan' # <-- NOVO: Tópico do laser
        heartbeat_topic = '/heartbeat'

        self.publisher = self.node.create_publisher(Twist, cmd_vel_topic, 10)
        self.odom_subscription = self.node.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.heartbeat_publisher = self.node.create_publisher(String, heartbeat_topic, 10)
        
        # --- NOVO: SUBSCRITOR PARA O SENSOR LASER ---
        self.scan_subscription = self.node.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data) # Perfil de QoS para sensores

        self.heartbeat_timer = self.node.create_timer(1.0, self.publish_heartbeat)

        self.current_pose = None
        self.current_yaw = 0.0
        self.front_distance = float('inf') # <-- NOVO: Armazena a distância frontal

        self.node.get_logger().info(f"'{self.namespace}' RobotDriver inicialized.")

    def scan_callback(self, msg):
        """Callback que é chamado sempre que uma nova leitura do laser é recebida."""
        # O TurtleBot3 tem 360 raios. O raio 0 é o que está virado para a frente.
        if len(msg.ranges) > 0:
            self.front_distance = msg.ranges[0]

    def is_block_in_front(self, contact_threshold=0.2):
        """
        Verifica se há um objeto (o bloco) em contacto com a frente do robô.
        A distância de contacto é ligeiramente maior que o raio do robô + raio do cilindro.
        """
        # Se a distância for muito pequena, considera-se contacto.
        # Também verificamos se não é 0.0, que muitas vezes indica um erro de leitura.
        return self.front_distance < contact_threshold and self.front_distance > 0.0
    def publish_heartbeat(self):
        if not self.is_failed:
            msg = String()
            msg.data = self.namespace
            self.heartbeat_publisher.publish(msg)

    def simulate_failure(self):
        self.node.get_logger().warn(f"SIMULATING ROBOT FAIL:'{self.namespace}'!")
        self.is_failed = True
        self.stop()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        orientation_q = self.current_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

    def get_velocity_to_goal(self, goal_pose, linear_speed, angular_speed, goal_tolerance):
        # Se o robô falhou, ele para e informa que NÃO chegou ao destino.
        if self.is_failed:
            return self.stop(), False # <-- CORREÇÃO: MUDADO DE True PARA False

        if self.current_pose is None:
            return None, False
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        goal_x = goal_pose[0]
        goal_y = goal_pose[1]

        distance_to_goal = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
        
        twist_msg = Twist()

        # Se já chegamos ao alvo (e não falhámos), para e informa que chegou.
        if distance_to_goal < goal_tolerance:
            return self.stop(), True

        # Lógica de Controlo Proporcional (inalterada)
        angle_to_goal = math.atan2(goal_y - current_y, goal_x - current_x)
        angle_error = angle_to_goal - self.current_yaw
        if angle_error > math.pi: angle_error -= 2 * math.pi
        elif angle_error < -math.pi: angle_error += 2 * math.pi
        twist_msg.angular.z = angular_speed * angle_error
        if abs(angle_error) < math.pi / 2:
            twist_msg.linear.x = linear_speed
        else:
            twist_msg.linear.x = 0.0
        
        return twist_msg, False

    def stop(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)
        return twist_msg