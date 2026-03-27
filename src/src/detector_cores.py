#!/usr/bin/env python3

from geometry_msgs.msg import Twist # Adicione este import no topo
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan 
import time


class DetectorCoresNode(Node):

    def __init__(self):
        super().__init__('detector_cores')

        self.bridge = CvBridge()
        self.distancia_seguranca = 2.0
        self.frente_direita = 1000
        self.frente_esquerda = 1000

        # Publishers
        self.pub_cor = self.create_publisher(String, '/cor_atual', 10)
        self.pub_contagem = self.create_publisher(String, '/contagem_cores', 10)
        
        # Publisher de movimento (O controle do robô)
        self.pub_cmd_vel = self.create_publisher(Twist, '/jetauto/cmd_vel', 10)

        # Subscriber da câmera
        self.sub_camera = self.create_subscription(
            Image,
            '/frente_camera/frente_camera_sensor/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        # Subscriber da odometria
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )
        
        # Subscriber do Lidar (Note o tópico correto do JetAuto)
        self.sub_laser = self.create_subscription(
            LaserScan,
            '/jetauto/lidar/scan',
            self.laser_callback,
            qos_profile_sensor_data
        )
        self.distancia_frente = 10.0

        # posição atual do robô
        self.robot_pos = None

        # contagem
        self.contagem = {
            'vermelho': 0,
            'verde': 0,
            'azul': 0
        }

        # lista de posições detectadas
        self.objetos_detectados = {
            'vermelho': [],
            'verde': [],
            'azul': []
        }

        # distância mínima para considerar novo objeto
        self.map_threshold = 1.5


    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.robot_pos = (x, y)


    def distancia(self, p1, p2):

        return np.sqrt(
            (p1[0] - p2[0])**2 +
            (p1[1] - p2[1])**2
        )


    def ja_detectado(self, cor):

        for pos in self.objetos_detectados[cor]:

            dist = self.distancia(self.robot_pos, pos)

            if dist < self.map_threshold:
                return True

        return False
        
    def laser_callback(self, msg):
        # O Lidar 360 tem muitos pontos. Vamos pegar a média dos pontos centrais (frente)
        # Em muitos Lidars, os índices centrais (ex: 0, 1, 359) são a frente.
        # Vamos pegar uma fatia pequena da frente:
        
        ranges = msg.ranges
        n = len(ranges)

        centro = n // 2

        frente = ranges[centro-11:centro+11]
        frente_esquerda = ranges[:centro+13]
        frente_direita = ranges[centro-13:]
        esquerda = ranges[centro+20:centro+50]
        direita = ranges[centro-50:centro-20]

        frente_valid = [d for d in frente if msg.range_min < d < msg.range_max]
        frente_valid_esquerda = [d for d in frente_esquerda if msg.range_min < d < msg.range_max]
        frente_valid_direita = [d for d in frente_direita if msg.range_min < d < msg.range_max]
        esquerda_valid = [d for d in esquerda if msg.range_min < d < msg.range_max]
        direita_valid = [d for d in direita if msg.range_min < d < msg.range_max]

        if frente_valid:
            self.distancia_frente = min(frente_valid)
            self.frente_esquerda = min(frente_valid_esquerda)
            self.frente_direita = min(frente_valid_direita)
        else:
            self.distancia_frente = msg.range_max

        if esquerda_valid:
            self.distancia_esquerda = max(esquerda_valid)
        else:
            self.distancia_esquerda = msg.range_max

        if direita_valid:
            self.distancia_direita = max(direita_valid)
        else:
            self.distancia_direita = msg.range_max


    def image_callback(self, msg):

        if self.robot_pos is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # altura, largura, _ = cv_image.shape
        # roi = cv_image[int(altura*0.6):altura, int(largura*0.3):int(largura*0.7)]

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # máscaras de cor
        mask_red1 = cv2.inRange(hsv, np.array([0,120,70]), np.array([10,255,255]))
        mask_red2 = cv2.inRange(hsv, np.array([170,120,70]), np.array([180,255,255]))
        mask_red = mask_red1 + mask_red2

        mask_green = cv2.inRange(hsv, np.array([36,50,50]), np.array([89,255,255]))
        mask_blue = cv2.inRange(hsv, np.array([90,50,50]), np.array([128,255,255]))

        area_red = cv2.countNonZero(mask_red)
        area_green = cv2.countNonZero(mask_green)
        area_blue = cv2.countNonZero(mask_blue)

        limite_area = 15000

        cor_atual = 'nenhuma'            
        if area_red > limite_area and area_red > area_green and area_red > area_blue:
            cor_atual = 'vermelho'

        elif area_green > limite_area and area_green > area_red and area_green > area_blue:
            cor_atual = 'verde'

        elif area_blue > limite_area and area_blue > area_red and area_blue > area_green:
            cor_atual = 'azul'


        # -------- CONTAGEM --------

        if cor_atual != 'nenhuma':

            if not self.ja_detectado(cor_atual):

                self.contagem[cor_atual] += 1

                self.objetos_detectados[cor_atual].append(self.robot_pos)

                self.get_logger().info(
                    f'Novo objeto {cor_atual.upper()} detectado '
                    f'na posição x={self.robot_pos[0]:.2f}, y={self.robot_pos[1]:.2f}'
                )


        # -------- PUBLICAÇÃO --------

        msg_cor = String()
        msg_cor.data = cor_atual
        self.pub_cor.publish(msg_cor)

        msg_contagem = String()
        msg_contagem.data = (
            f"Verde: {self.contagem['verde']} | "
            f"Vermelho: {self.contagem['vermelho']} | "
            f"Azul: {self.contagem['azul']}"
        )

        self.pub_contagem.publish(msg_contagem)

	#----------------------------------------------------------- a partir daqui está a lógica de navegação no labirinto
        vel_msg = Twist()
        distancia_seguranca = self.distancia_seguranca # distancia de segurança do lidar
        speed = 2.1 * min(0.35, 0.375 * self.distancia_frente)
        erro = min (2.0, 0.97/(self.distancia_frente))
        fator_curva = 0.25
        velocidade_curva = fator_curva * speed
        
        if cor_atual == 'vermelho':
            self.get_logger().info('Esquerda! (Cor Vermelha)')
            vel_msg.linear.x = 0.8*velocidade_curva
            vel_msg.angular.z = (0.8/self.distancia_frente)
        elif cor_atual == 'verde':
            self.get_logger().info('Direita! (Cor Verde)')
            vel_msg.linear.x = velocidade_curva
            vel_msg.angular.z = -erro
        elif self.frente_esquerda < distancia_seguranca and self.frente_direita > distancia_seguranca:
            self.get_logger().info("Virando para esquerda (mais espaço)")
            vel_msg.linear.x = min(0.3, velocidade_curva)
            vel_msg.angular.z = erro
        elif self.frente_direita < distancia_seguranca and self.frente_esquerda > distancia_seguranca:
            self.get_logger().info("Virando para direita (mais espaço)")
            vel_msg.linear.x = min(0.3, velocidade_curva)
            vel_msg.angular.z = -(0.8/self.frente_direita)
        elif self.distancia_frente < distancia_seguranca:
            self.get_logger().warn(f"Obstáculo a {self.distancia_frente:.2f}m. Parando!")
            if self.distancia_esquerda > self.distancia_direita:
                self.get_logger().info("Virando para esquerda (mais espaço)")
                vel_msg.linear.x = min(0.3, velocidade_curva)
                vel_msg.angular.z = erro
            else:
                self.get_logger().info("Virando para direita (mais espaço)")
                vel_msg.linear.x = min(0.3, velocidade_curva)
                vel_msg.angular.z = -erro

            self.pub_cmd_vel.publish(vel_msg)

        else: # 'nenhuma' cor detectada
            self.get_logger().info('Procurando cores...')
            if  2.0 < self.distancia_frente <= 3.0:
                vel_msg.linear.x = 0.4 * speed
            elif 3.0 < self.distancia_frente <= 3.5:
                vel_msg.linear.x = 0.45 * speed
            elif 3.5 < self.distancia_frente <= 4.0:
                vel_msg.linear.x = 0.6 * speed
            else:
                vel_msg.linear.x = speed

            vel_msg.angular.z = 0.00

        
        self.pub_cmd_vel.publish(vel_msg) # PUBLICAÇÃO ÚNICA: Envia o comando final decidido acima

def main(args=None):

    rclpy.init(args=args)

    node = DetectorCoresNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
