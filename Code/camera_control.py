#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import cv2 
import numpy as np

class CameraControlNode(Node):
    def __init__(self):
        super().__init__("camera_control_node")
        self.get_logger().info("I am camera_control_node")
        self.object_names_subscription = self.create_subscription(String, 'object_detected', self.object_names_callback, 10)
        self.confidence_scores_subscription = self.create_subscription(Float32MultiArray, 'confidence_score', self.confidence_scores_callback, 10)
        self.coordinates_subscription = self.create_subscription(Float32MultiArray, 'object_coordinates', self.coordinates_callback, 10)
        self.motion_pan_camera_publisher = self.create_publisher(String, 'motion_pan_camera', 10)
        self.motion_til_camera_publisher = self.create_publisher(String, 'motion_til_camera', 10)
        self.final_object_publisher = self.create_publisher(String, 'final_detected_object', 10)
        self.current_object = None
        self.current_coordinates = None
        self.is_kid_detected = False  
        self.is_kid_centered = False  
        self.timer = None 
        self.changes_detected = False  
        self.kid_detected = False
        self.object_to_detect = "adult"
        self.current_pan_command = "00"
        self.current_til_command = "0000"
        self.frame_width = 640
        self.frame_height = 480
        self.center_threshold = 20  
        self.detection_count = 0 
        self.detection_threshold = 4 
        self.searching = False 
        self.dominant_color = None  
        self.consecutive_model_detections = 0  
        self.model_update_threshold = 5  
        self.dangerous_objects = ["glass marbles", "knife", "coin", "outlet", "pill", "scissors"]

    def publish_commands(self):
        if not self.changes_detected:
            return
        pan_msg = String()
        pan_msg.data = self.current_pan_command
        self.get_logger().info(f"[DEBUG] Publicado PAN: {self.current_pan_command}")
        til_msg = String()
        til_msg.data = self.current_til_command
        self.get_logger().info(f"[DEBUG] Publicado TIL: {self.current_til_command}")

    def start_timer(self):
        if not self.timer:
            self.timer = self.create_timer(0.5, self.publish_camera_commands)

    def stop_timer(self):
        if self.timer:
            self.timer.cancel()
            self.timer = None

    def publish_camera_commands(self):
        if self.searching: 
            pan_msg = String()
            pan_msg.data = self.current_pan_command
            til_msg = String()
            til_msg.data = self.current_til_command
            return 
        if not self.kid_detected:
            self.stop_timer()
            return
        pan_msg = String()
        pan_msg.data = self.current_pan_command
        til_msg = String()
        til_msg.data = self.current_til_command
        pan_msg = String()
        pan_msg.data = self.current_pan_command
        til_msg = String()
        til_msg.data = self.current_til_command 

    def object_names_callback(self, msg):
        self.current_object = msg.data.lower()
        if self.current_object in self.dangerous_objects:
            self.get_logger().warn(f"Objeto peligroso detectado: {self.current_object}")
            dangerous_object_msg = String()
            dangerous_object_msg.data = self.current_object
            self.final_object_publisher.publish(dangerous_object_msg)
            self.get_logger().info(f"Objeto peligroso enviado al tópico: {self.current_object}")
        else:
            self.get_logger().info(f"Objeto detectado no peligroso o no detectado: {self.current_object}")
            safe_message = String()
            safe_message.data = "Todo está bien"
            self.final_object_publisher.publish(safe_message)
            self.get_logger().info("Mensaje enviado al tópico: Todo está bien")

    def update_opencv_features(self, frame, bbox):
        x1, y1, x2, y2 = [int(coord) for coord in bbox]
        x1 = max(0, min(x1, self.frame_width - 1))
        x2 = max(0, min(x2, self.frame_width - 1))
        y1 = max(0, min(y1, self.frame_height - 1))
        y2 = max(0, min(y2, self.frame_height - 1))
        if x1 >= x2 or y1 >= y2:
            self.get_logger().warning("Bounding box no válido para actualizar características.")
            return
        roi = frame[y1:y2, x1:x2]
        average_color = cv2.mean(roi)[:3]
        self.dominant_color = tuple(map(int, average_color))
        self.get_logger().info(f"[DEBUG] Color dominante actualizado: {self.dominant_color}")

    def start_searching(self):
        if not self.searching:
            self.searching = True
        self.current_pan_command = "10"  
        self.current_til_command = "0000" 

    def confidence_scores_callback(self, msg):
        confidence_score = round(msg.data[0], 5)

    def coordinates_callback(self, msg):
        self.current_coordinates = [round(coord, 5) for coord in msg.data]

    def adjust_camera(self):
        if not self.kid_detected:
            self.get_logger().info("[DEBUG] No hay detección confirmada. No se ajustará la cámara.")
            return
        if not self.current_coordinates:
            self.get_logger().warning("No hay coordenadas disponibles para ajustar la cámara.")
            return
        x1, y1, x2, y2 = [int(coord) for coord in self.current_coordinates]
        if x1 >= x2 or y1 >= y2: 
            self.get_logger().warning("El bounding box recibido no es válido.")
            return
        object_center_x = (x1 + x2) / 2
        object_center_y = (y1 + y2) / 2
        frame_center_x = self.frame_width / 2
        frame_center_y = self.frame_height / 2
        pan_command = "00"
        til_command = "0000"
        error_margin = 50  
        if abs(object_center_x - frame_center_x) > self.center_threshold + error_margin:
            pan_command = "10" if object_center_x < frame_center_x else "01"
            direction = "derecha" if pan_command == "01" else "izquierda"
            self.get_logger().info(f"Mover {direction} para centrar horizontalmente.")
        if abs(object_center_y - frame_center_y) > self.center_threshold + error_margin:
            til_command = "0001" if object_center_y < frame_center_y else "0010"
            direction = "arriba" if til_command == "0001" else "abajo"
            self.get_logger().info(f"Mover {direction} para centrar verticalmente.")
        if (self.current_pan_command != pan_command) or (self.current_til_command != til_command):
            self.current_pan_command = pan_command
            self.current_til_command = til_command
            self.changes_detected = True  
            self.start_timer() 
        else:
            self.get_logger().info("[DEBUG] La cámara ya está centrada.")
            self.changes_detected = False

    def process_opencv(self, frame, bbox):
        x1, y1, x2, y2 = [int(coord) for coord in bbox]  
        x1 = max(0, min(x1, self.frame_width - 1))
        x2 = max(0, min(x2, self.frame_width - 1))
        y1 = max(0, min(y1, self.frame_height - 1))
        y2 = max(0, min(y2, self.frame_height - 1))
        if x1 >= x2 or y1 >= y2:  
            self.get_logger().warning("Bounding box no válido para procesar.")
            return False
        roi = frame[y1:y2, x1:x2]
        average_color = cv2.mean(roi)[:3]
        dominant_color = tuple(map(int, average_color))
        self.get_logger().info(f"[DEBUG] Color dominante detectado: {dominant_color}")
        if self.dominant_color is None:
            self.dominant_color = dominant_color
            self.get_logger().info("[DEBUG] Color dominante inicializado.")
        color_difference = np.linalg.norm(np.array(self.dominant_color) - np.array(dominant_color))
        self.get_logger().info(f"[DEBUG] Diferencia de color: {color_difference}")
        return color_difference < 50 

def main(args=None):
    rclpy.init(args=args)
    node = CameraControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
