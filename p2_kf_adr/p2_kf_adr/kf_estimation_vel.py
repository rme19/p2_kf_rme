import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped

import numpy as np
import math


from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, generate_noisy_measurement_2
from .filters.kalman_filter import KalmanFilter_2
from .visualization import Visualizer

class KalmanFilterPureNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_pure_node')

        # TODO: Initialize 6D state and covariance
        self.initial_state = np.zeros(6)
        initial_covariance = np.eye(6) * 0.1

        self.prev_time = None

        self.kf = KalmanFilter_2(self.initial_state, initial_covariance)

        self.visualizer = Visualizer()


        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf2_estimate',
            10
        )

    def odom_callback(self, msg):
        # TODO: Extract position, orientation, velocities from msg
        # TODO: Run predict() and update() of KalmanFilter_2
        # TODO: Publish estimated full state
        # Inicializamos la medición de la posición actual
        current_pose = odom_to_pose2D(msg)
        # Normalizamos la medida (solo cogemos las dimensiones de la posición del initial_state)
        self.normalized_pose = np.array(get_normalized_pose2D(self.initial_state[:3] , current_pose))

        # Entradas provenientes de la odometría
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        # Sacamos su norma
        v = np.sqrt(linear.x**2 + linear.y**2)
        omega = np.sqrt(angular.x**2 + angular.y**2 + angular.z**2)
        # Las agrupamos en el vector u
        u = np.array([v,omega])

        # Añadimos ruido
        z = generate_noisy_measurement_2(self.normalized_pose, linear.x, linear.y, omega)

        # Sacamos el momento actual para calcular dt
        t_act=self.get_clock().now().nanoseconds
        if self.prev_time:
            delta_t=(t_act-self.prev_time)/1e9
        else:
            delta_t=0.1

        #Ejecutamos predicción para estimar vector de media y matriz de covarianza
        mu, sigma= self.kf.predict(u, delta_t)
        
        #Ejecutamos la actualización para estimar vector de media y matriz de covarianza
        mu, sigma= self.kf.update(z)

        # Creamos y rellenamos el mensaje que vamos a publicar en /kf_estimate
        msg_pub = PoseWithCovarianceStamped()
        msg_pub.header.stamp = self.get_clock().now().to_msg()
        msg_pub.header.frame_id = 'map'
        msg_pub.pose.pose.position.x = mu[0]
        msg_pub.pose.pose.position.y = mu[1]
        msg_pub.pose.pose.orientation.z = np.sin(mu[2]/2)
        msg_pub.pose.pose.orientation.w = np.cos(mu[2]/2)
        msg_pub.pose.covariance = [0.0] * 36  # inicializa
        msg_pub.pose.covariance[0] = sigma[0, 0]  # x
        msg_pub.pose.covariance[1] = sigma[0, 1]  # xy
        msg_pub.pose.covariance[6] = sigma[1, 0]  # yx
        msg_pub.pose.covariance[7] = sigma[1, 1]  # y
        msg_pub.pose.covariance[35] = sigma[2, 2]  # yaw (θ)

        # Publicamos el mensaje
        self.publisher.publish(msg_pub)

        # Actualizamos el tiempo anterior
        self.prev_time=t_act

        self.visualizer.update(self.normalized_pose , mu , sigma , step="update")

        return mu, sigma

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterPureNode()
    rclpy.spin(node)
    rclpy.shutdown()

