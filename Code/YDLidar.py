#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import PyLidar3
import numpy as np
import matplotlib.pyplot as plt 
from matplotlib.animation import FuncAnimation 

class YDLidarNode(Node):
    def __init__(self):
        super().__init__("ydlidar_node")
        self.get_logger().info("YDlidar_node initialized")
        self.warning_publisher = self.create_publisher(String, 'Warning_YDLidar', 10)
        self.port = "/dev/ttyUSB0" 
        self.lidar = PyLidar3.YdLidarX4(self.port)
        if self.lidar.Connect():
            self.get_logger().info("LIDAR connected successfully")
            device_info = self.lidar.GetDeviceInfo()  
            self.get_logger().info(str(device_info))  
        else:
            self.get_logger().error("Failed to connect to LIDAR")
            return
        self.scan_generator = self.lidar.StartScanning()
        if self.visualizar:
            self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
            self.ax.set_title('Distribución de distancias por ángulo (en tiempo real)')
            self.ax.set_theta_zero_location('N') 
            self.ax.set_theta_direction(-1)
            self.ax.set_xlabel('Angulo (radianes)')
            self.ax.set_ylabel('Distancia')
            self.ani = FuncAnimation(self.fig, self.update_plot, interval=500)
            plt.show()

    def filter_and_detect_anomalies(self, angles, distances):
        filtered_angles = []
        filtered_distances = []
        threshold = 15  
        anomalous_points = []
        mean_distance = np.mean(distances) if distances else 0
        for i in range(2, len(distances) - 2):
            neighbors = distances[i-2:i+3]
            avg_distance = np.mean(neighbors)
            if abs(distances[i] - avg_distance) < threshold:
                filtered_angles.append(angles[i])
                filtered_distances.append(distances[i])
            if distances[i] > 3 * mean_distance:
                anomalous_points.append((angles[i], distances[i]))
        msg = String()
        if anomalous_points:
            msg.data = "Puerta o ventana abierta"
        else:
            msg.data = "Todo está bien"
        self.warning_publisher.publish(msg)
        self.get_logger().info(f"{msg.data}")
        return filtered_angles, filtered_distances, anomalous_points

    def update_plot(self, frame):
        try:
            data = next(self.scan_generator)  
            angles_45_160 = []
            distances_45_160 = []
            angles_190_315 = []
            distances_190_315 = []
            for angle in range(45, 161):
                if angle in data:
                    distance = data[angle]
                    if distance > 0: 
                        angles_45_160.append(np.radians(angle))  
                        distances_45_160.append(distance)
            for angle in range(190, 316):
                if angle in data:
                    distance = data[angle]
                    if distance > 0: 
                        angles_190_315.append(np.radians(angle))  
                        distances_190_315.append(distance)
            filtered_angles_45_160, filtered_distances_45_160, anomalies_45_160 = self.filter_and_detect_anomalies(
                angles_45_160, distances_45_160
            )
            filtered_angles_190_315, filtered_distances_190_315, anomalies_190_315 = self.filter_and_detect_anomalies(
                angles_190_315, distances_190_315
            )
            self.ax.clear()
            self.ax.set_title('Distribución de distancias por ángulo (en tiempo real)')
            self.ax.set_theta_zero_location('N') 
            self.ax.set_theta_direction(-1) 
            self.ax.set_xlabel('Angulo (radianes)')
            self.ax.set_ylabel('Distancia')
            self.ax.scatter(filtered_angles_45_160, filtered_distances_45_160, c='blue', label='Rango 45°-160°')
            self.ax.scatter(filtered_angles_190_315, filtered_distances_190_315, c='red', label='Rango 190°-315°')
            self.ax.legend()
            if anomalies_45_160 or anomalies_190_315:
                self.get_logger().warn("Ventana o puerta abierta detectada.")
                for angle, distance in anomalies_45_160 + anomalies_190_315:
                    self.get_logger().warn(f"Ángulo: {np.degrees(angle):.1f}° - Distancia: {distance:.1f}")
            else:
                self.get_logger().info("Todo está bien.")
        except StopIteration:
            self.get_logger().error("LIDAR scan stopped unexpectedly")

    def close_lidar(self):
        self.lidar.StopScanning()
        self.lidar.Disconnect()
        self.get_logger().info("LIDAR disconnected")

def main(args=None):
    rclpy.init(args=args)
    node = YDLidarNode()
    try:
        rclpy.spin(node)
    finally:
        node.close_lidar()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
