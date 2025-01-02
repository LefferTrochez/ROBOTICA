#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading 
import time 
from std_msgs.msg import Bool

class ControlArduinoNode(Node):
    def __init__(self):
        super().__init__("control_arduino_node")
        self.modo_operacion = "1"
        self.warning = "0"
        self.motion_control_pan = "00"
        self.motion_control_til = "0000"
        self.led_boca = "1"
        self.led_orejas = "1"
        self.led_ojos = "111"
        self.last_received_data = False 
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.motion_pan_subscription = self.create_subscription(String, 'motion_pan_arduino', self.motion_pan_callback, 10)
        self.motion_til_subscription = self.create_subscription(String, 'motion_til_arduino', self.motion_til_callback, 10)
        self.mode_operation_subscription = self.create_subscription(String, 'mode_operation_arduino', self.mode_operation_callback, 10)
        self.leds_rgb_subscription = self.create_subscription(String, 'LEDsRGB_arduino', self.leds_rgb_callback, 10)
        self.warning_arduino_subscription = self.create_subscription(String, 'warning_arduino', self.warning_arduino_callback, 10)
        self.motion_pan_web_page_subscription = self.create_subscription(String, 'motion_pan_web_page', self.motion_pan_web_page_callback, 10)
        self.motion_til_web_page_subscription = self.create_subscription(String, 'motion_til_web_page', self.motion_til_web_page_callback, 10)
        self.motion_pan_camera_subscription = self.create_subscription(String, 'motion_pan_camera', self.motion_pan_camera_callback, 10)
        self.motion_til_camera_subscription = self.create_subscription(String, 'motion_til_camera', self.motion_til_camera_callback, 10)
        self.control_serial_publisher = self.create_publisher(Bool, 'control_serial_access', 10)
        threading.Thread(target=self.send_periodic_messages, daemon=True).start()
        self.send_serial_message()

    def send_periodic_messages(self):
        while True:
            if self.last_received_data:  
                self.send_serial_message()
                self.last_received_data = False 
            time.sleep(1) 

    def send_serial_message(self):
        if self.serial_port.is_open:
            control_msg = Bool()
            control_msg.data = True  
            self.control_serial_publisher.publish(control_msg)
            message = f"{self.modo_operacion},{self.warning},{self.motion_control_pan},{self.motion_control_til},{self.led_boca},{self.led_orejas},{self.led_ojos}"
            self.serial_port.write(message.encode())
            self.get_logger().info(f"Enviado al Arduino: {message}")
            time.sleep(0.1)
            control_msg.data = False 
            self.control_serial_publisher.publish(control_msg)

    def motion_pan_camera_callback(self, msg):
        self.motion_control_pan = msg.data
        self.last_received_data = True  

    def motion_til_camera_callback(self, msg):
        self.motion_control_til = msg.data
        self.last_received_data = True

    def motion_pan_callback(self, msg):
        self.motion_control_pan = msg.data
        self.last_received_data = True  

    def motion_til_callback(self, msg):
        self.motion_control_til = msg.data
        self.last_received_data = True 

    def motion_pan_web_page_callback(self, msg):
        self.motion_control_pan = msg.data
        self.last_received_data = True  

    def motion_til_web_page_callback(self, msg):
        self.motion_control_til = msg.data
        self.last_received_data = True 

    def mode_operation_callback(self, msg):
        self.modo_operacion = "1" if msg.data == '1' else "0"
        self.motion_control_pan = "00" 
        self.motion_control_til = "0000"
        self.last_received_data = True  

    def warning_arduino_callback(self, msg):
        self.warning = "1" if msg.data == '1' else "0"
        self.motion_control_pan = "00"  
        self.motion_control_til = "0000"  
        self.last_received_data = True  

    def leds_rgb_callback(self, msg):
        try:
            led_boca, led_orejas, led_ojos = msg.data.split(',')
            self.led_boca = led_boca
            self.led_orejas = led_orejas
            self.led_ojos = led_ojos
            self.last_received_data = True  
        except ValueError:
            self.get_logger().error(f"[TOPIC: LEDsRGB_arduino] Formato incorrecto: {msg.data}")

    def close_serial_port(self):
        if self.serial_port.is_open:
            self.serial_port.close()

def main(args=None):
    rclpy.init(args=args)
    node = ControlArduinoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_serial_port()
        rclpy.shutdown()

if __name__ == "__main__":
    main()