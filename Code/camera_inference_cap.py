#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image
import cv2
import numpy as np
from ultralytics import YOLO

class CameraInferenceCapNode(Node):
    def __init__(self):
        super().__init__("camera_inference_cap_node")
        self.get_logger().info("I am camera_inference_cap_node")
        self.model = YOLO('best_3.pt')
        self.frame_web_page_publisher = self.create_publisher(String, 'frame_web_page', 10)
        self.object_names_publisher = self.create_publisher(String, 'object_detected', 10)
        self.confidence_scores_publisher = self.create_publisher(Float32MultiArray, 'confidence_score', 10)
        self.coordinates_publisher = self.create_publisher(Float32MultiArray, 'object_coordinates', 10)
        self.cap = self.open_camera()
        self.timer = self.create_timer(0.01, self.process_frame)

    def open_camera(self):
        gst_str = ("nvarguscamerasrc ! "
                   "video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1 ! "
                   "nvvidconv ! video/x-raw, format=BGRx ! "
                   "videoconvert ! video/x-raw, format=BGR ! appsink max-buffers=1 drop=True sync=false")
        cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            self.get_logger().error("Error: No se pudo abrir la cámara.")
            return None
        return cap

    def process_frame(self):
        if self.cap is None:
            return
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Error: No se pudo obtener el frame.")
            return
        results = self.model(frame)
        _, buffer = cv2.imencode('.jpg', frame)
        jpeg_bytes = buffer.tobytes()
        img_msg = String()
        img_msg.data = jpeg_bytes.hex()  
        self.frame_web_page_publisher.publish(img_msg)
        if results and results[0].boxes:
            object_names = String()
            confidence_scores = Float32MultiArray()
            coordinates = Float32MultiArray()
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
                    conf = float(box.conf.item())
                    cls = int(box.cls.item())
                    object_name = self.get_class_name(cls)
                    object_names.data = object_name
                    self.object_names_publisher.publish(object_names)
                    confidence_scores.data = [conf]
                    self.confidence_scores_publisher.publish(confidence_scores)
                    coordinates.data = [x1, y1, x2, y2]
                    self.coordinates_publisher.publish(coordinates)

    def get_class_name(self, class_id):
        class_names = ['kid', 'glass marbles', 'knife', 'cats', 'coin', 
                       'outlet', 'pill', 'dogs', 'adult', 'scissors']
        return class_names[class_id]

def main(args=None):
    rclpy.init(args=args)
    node = CameraInferenceCapNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
