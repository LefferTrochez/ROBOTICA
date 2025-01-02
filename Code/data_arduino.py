#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import serial

class DataArduinoNode(Node):
    def __init__(self):
        super().__init__("sensors_data_arduino_node")
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.sensor_data_publisher = self.create_publisher(String, 'sensor_data_arduino', 10)
        self.allow_serial_access = True 
        self.serial_access_subscription = self.create_subscription(Bool,'control_serial_access',self.serial_access_callback,10)
        self.ambient_light_publisher = self.create_publisher(Bool, 'ambient_light_sensor', 10)
        self.timer = self.create_timer(0.1, self.publish_sensor_data)

    def serial_access_callback(self, msg):
        if isinstance(msg.data, bool):  
            self.allow_serial_access = not msg.data  
            self.get_logger().info(f"Acceso serial {'permitido' if self.allow_serial_access else 'bloqueado'}.")
        else:
            self.get_logger().error("Mensaje recibido en control_serial_access no es booleano.")

    def publish_sensor_data(self):
        if self.allow_serial_access and self.serial_port.is_open:
            try:
                data = self.serial_port.readline().decode('utf-8').strip()
                if data:
                    msg = String()
                    msg.data = data
                    self.sensor_data_publisher.publish(msg)
                    self.get_logger().info(f"Publicado en sensor_data_arduino: {msg.data}")
                    self.process_ambient_light_data(data)
            except UnicodeDecodeError as e:
                self.get_logger().warning(f"Error de decodificación: {e}")
            except serial.SerialException as e:
                self.get_logger().error(f"Error leyendo desde el puerto serial: {e}")
        else:
            self.get_logger().debug(f"Acceso serial bloqueado: {not self.allow_serial_access}")

    def process_ambient_light_data(self, raw_data):
        try:
            data = raw_data.split(',')
            ambient_light = float(data[6]) 
            light_msg = Bool()
            light_msg.data = ambient_light < 16
            self.ambient_light_publisher.publish(light_msg)
            state = "True (activado)" if light_msg.data else "False (desactivado)"
            self.get_logger().info(f"ambient_light_sensor publicado: {state} (valor: {ambient_light})")
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Error procesando datos del sensor de luz: {e}")

    def close_serial_port(self):
        if self.serial_port.is_open:
            self.serial_port.close()

def main(args=None):
    rclpy.init(args=args)
    node = DataArduinoNode()
    rclpy.spin(node)
    node.close_serial_port()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
