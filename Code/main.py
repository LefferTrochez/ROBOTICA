#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import threading
import time

class MainNode(Node):
    def __init__(self):
        super().__init__("main_node")
        self.get_logger().info("I am main_node")
        self.last_warning_message = "Todo está bien."
        self.warning = "0"
        self.LED_boca = '0'
        self.LED_orejas = '0'
        self.LED_ojos = '110'
        self.modo_operacion = "0" 
        self.estado_buzzer = "0"  
        self.threshold_decibels = 40.0
        self.threshold_temperature = 30.0
        self.threshold_mq135 = 200.0
        self.threshold_sensor_fuego = 3.35
        self.threshold_ambient_light = 16.0
        self.threshold_thermal_camera = 55.0  
        self.earthquake_status = 0
        self.decibels_samples = []
        self.temperature_samples = []
        self.mq135_samples = []
        self.sensor_fuego_samples = []
        self.proximity_samples = []
        self.ambient_light_samples = []
        self.thermal_pixels_samples = []
        self.update_decibels = False
        self.update_temperature = False
        self.update_mq135 = False
        self.update_sensor_fuego = False
        self.current_dangerous_object = None 
        self.is_alert_active = False 
        self.alert_timer = None  
        self.detected_object_main_node_subscription = self.create_subscription(String, 'final_detected_object', self.detected_object_callback, 10)
        self.sensor_data_main_node_subscription = self.create_subscription(String, 'sensor_data_arduino', self.sensor_data_callback, 10)
        self.mode_operation_web_page_subscription = self.create_subscription(String, 'mode_operation_web_page', self.mode_operation_web_page_callback, 10)
        self.stop_warning_web_page_subscription = self.create_subscription(String, 'stop_warning_web_page', self.stop_warning_web_page_callback, 10)
        self.ydlidar_warning_subscription = self.create_subscription(String,'Warning_YDLidar',self.ydlidar_warning_callback,10)
        self.motion_pan_arduino_main_node_publisher = self.create_publisher(String, 'motion_pan_arduino', 10)
        self.motion_til_arduino_main_node_publisher = self.create_publisher(String, 'motion_til_arduino', 10)
        self.mode_operation_arduino_main_node_publisher = self.create_publisher(String, 'mode_operation_arduino', 10)
        self.LEDsRGB_arduino_main_node_publisher = self.create_publisher(String, 'LEDsRGB_arduino', 10)
        self.warning_arduino_main_node_publisher = self.create_publisher(String, 'warning_arduino', 10)
        self.warning_web_page_main_node_publisher = self.create_publisher(String, 'warning_web_page', 10)
        self.sensor_values_web_page_publisher = self.create_publisher(String, 'sensor_values_web_page', 10)
        threading.Thread(target=self.publish_sensor_values, daemon=True).start()
        threading.Thread(target=self.publish_led_values, daemon=True).start()
        self.warning_web_page_main_node_publisher.publish(String(data="Todo está bien."))
        self.get_logger().info("Todo está bien.")
        threading.Thread(target=self.monitor_sensor_data, daemon=True).start()

    def sensor_data_callback(self, msg):
        try:
            data = msg.data.split(',')
            data[9:] = fix_data_format(data[9:])
            modo_operacion_recibido = data[0]
            estado_buzzer_recibido = data[1]
            self.earthquake_status = int(float(data[2]))
            proximity = int(data[3])
            decibels = float(data[4])
            temperature = float(data[5])
            ambient_light = float(data[6])
            mq135_value = float(data[7])
            sensor_fuego = float(data[8])
            pixels = list(map(float, data[9:73]))
            if modo_operacion_recibido != self.modo_operacion:
                self.modo_operacion = modo_operacion_recibido
                self.publish_modo_operacion()
            if estado_buzzer_recibido != self.estado_buzzer:
                self.estado_buzzer = estado_buzzer_recibido
                self.publish_estado_buzzer()
                if self.estado_buzzer == "2":
                    self.marcar_sensor_para_actualizacion()
            self.add_sensor_samples(proximity, decibels, temperature, ambient_light, mq135_value, sensor_fuego, pixels)
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Error al procesar sensor_data: {e}")
            self.get_logger().error(f"Datos recibidos: {msg.data}")
        self.add_sensor_samples(proximity, decibels, temperature, ambient_light, mq135_value, sensor_fuego, pixels)

    def add_sensor_samples(self, proximity, decibels, temperature, ambient_light, mq135_value, sensor_fuego, pixels):
        if len(self.proximity_samples) >= 10:
            self.proximity_samples.pop(0)
        if len(self.decibels_samples) >= 10:
            self.decibels_samples.pop(0)
        if len(self.temperature_samples) >= 10:
            self.temperature_samples.pop(0)
        if len(self.mq135_samples) >= 10:
            self.mq135_samples.pop(0)
        if len(self.sensor_fuego_samples) >= 10:
            self.sensor_fuego_samples.pop(0)
        if len(self.ambient_light_samples) >= 10:
            self.ambient_light_samples.pop(0)
        if len(self.thermal_pixels_samples) >= 10:
            self.thermal_pixels_samples.pop(0)
        self.proximity_samples.append(proximity)
        self.decibels_samples.append(decibels)
        self.temperature_samples.append(temperature)
        self.mq135_samples.append(mq135_value)
        self.sensor_fuego_samples.append(sensor_fuego)
        self.ambient_light_samples.append(ambient_light)
        self.thermal_pixels_samples.append(max(pixels))

    def publish_sensor_values(self):
        while True:
            if self.decibels_samples and self.temperature_samples and self.mq135_samples and self.thermal_pixels_samples:
                message = String()
                message.data = f"{self.decibels_samples[-1]},{self.temperature_samples[-1]},{self.thermal_pixels_samples[-1]},{self.mq135_samples[-1]}"
                self.sensor_values_web_page_publisher.publish(message)
            time.sleep(1) 

    def monitor_sensor_data(self):
        while True:
            self.evaluate_sensor_conditions()
            time.sleep(1)

    def evaluate_sensor_conditions(self):
        warning_arduino = "0"  
        warning_message = ""
        if self.earthquake_status == 1:
           pass
        if self.thermal_pixels_samples and any(value > self.threshold_thermal_camera for value in self.thermal_pixels_samples):
            warning_arduino = "1"
            warning_message += "La cámara térmica detecta alta temperatura. "
        if self.proximity_samples and all(value == 0 for value in self.proximity_samples):
            warning_arduino = "1"
            warning_message += "Tengo un obstáculo frente a mí. "
        if self.decibels_samples and all(value > self.threshold_decibels or math.isnan(value) for value in self.decibels_samples):
            warning_arduino = "1"
            warning_message += "Hay mucho ruido. "
        if self.temperature_samples and all(value > self.threshold_temperature for value in self.temperature_samples):
            warning_arduino = "1"
            warning_message += "La temperatura ambiente es muy alta. "
        if self.mq135_samples and all(value > self.threshold_mq135 for value in self.mq135_samples):
            warning_arduino = "1"
            warning_message += "Hay gas en el ambiente. "
        if self.sensor_fuego_samples and all(value < self.threshold_sensor_fuego for value in self.sensor_fuego_samples):
            warning_arduino = "1"
            warning_message += "Hay fuego en el ambiente. "
        if warning_message != self.last_warning_message:
            self.warning_web_page_main_node_publisher.publish(String(data=warning_message if warning_message else "Todo está bien."))
            self.get_logger().info(warning_message if warning_message else "Todo está bien.")
            self.last_warning_message = warning_message  
        if warning_arduino != self.warning:
            self.warning_arduino_main_node_publisher.publish(String(data=warning_arduino))
            self.get_logger().info(f"Advertencia enviada al Arduino: {warning_arduino}")
            self.warning = warning_arduino  

    def marcar_sensor_para_actualizacion(self):
        if any(value > self.threshold_decibels for value in self.decibels_samples):
            self.update_decibels = True
        if any(value > self.threshold_temperature for value in self.temperature_samples):
            self.update_temperature = True
        if any(value > self.threshold_mq135 for value in self.mq135_samples):
            self.update_mq135 = True
        if any(value < self.threshold_sensor_fuego for value in self.sensor_fuego_samples):
            self.update_sensor_fuego = True

    def reset_alert(self):
        self.is_alert_active = False
        self.current_dangerous_object = None
        self.warning = "0"
        self.warning_arduino_main_node_publisher.publish(String(data=self.warning))
        self.warning_web_page_main_node_publisher.publish(String(data="Todo está bien."))
        self.get_logger().info("Mensaje enviado a la página web: Todo está bien.")

    def mode_operation_web_page_callback(self, msg):
        self.get_logger().info("Callback activado: modo_operation_web_page_callback") 
        modo_msg = String()
        modo_msg.data = msg.data  
        self.get_logger().info(f"Recibido modo de operación desde la web: {msg.data}") 
        self.mode_operation_arduino_main_node_publisher.publish(modo_msg)
        self.get_logger().info(f"Modo de operación enviado al Arduino: {modo_msg.data}")

    def stop_warning_web_page_callback(self, msg):
        self.get_logger().info("Callback activado: stop_warning_web_page_callback")  
        buzzer_msg = String()
        buzzer_msg.data = msg.data  
        self.get_logger().info(f"Recibido comando de buzzer desde la web: {msg.data}") 
        self.warning_arduino_main_node_publisher.publish(buzzer_msg)
        self.get_logger().info(f"Comando de buzzer enviado al Arduino: {buzzer_msg.data}")
        if self.decibels_samples and all(value > self.threshold_decibels for value in self.decibels_samples):
            max_noise = max(self.decibels_samples)
            self.threshold_decibels = max_noise + 2
            self.get_logger().info(f"Umbral de decibeles actualizado a: {self.threshold_decibels}")
        elif self.temperature_samples and all(value > self.threshold_temperature for value in self.temperature_samples):
            max_temp = max(self.temperature_samples)
            self.threshold_temperature = max_temp + 2
            self.get_logger().info(f"Umbral de temperatura actualizado a: {self.threshold_temperature}")
        elif self.mq135_samples and all(value > self.threshold_mq135 for value in self.mq135_samples):
            max_gas = max(self.mq135_samples)
            self.threshold_mq135 = max_gas + 2
            self.get_logger().info(f"Umbral de MQ135 actualizado a: {self.threshold_mq135}")
        elif self.sensor_fuego_samples and all(value < self.threshold_sensor_fuego for value in self.sensor_fuego_samples):
            min_fire = min(self.sensor_fuego_samples)
            self.threshold_sensor_fuego = min_fire - 2
            self.get_logger().info(f"Umbral del sensor de fuego actualizado a: {self.threshold_sensor_fuego}")
        elif self.thermal_pixels_samples and any(value > self.threshold_thermal_camera for value in self.thermal_pixels_samples):
            max_pixel_temp = max(self.thermal_pixels_samples)
            self.threshold_thermal_camera = max_pixel_temp + 2
            self.get_logger().info(f"Umbral de la cámara térmica actualizado a: {self.threshold_thermal_camera}")

    def ydlidar_warning_callback(self, msg):
        if msg.data == "Puerta o ventana abierta":
            self.warning = "1"
            warning_message = "Alguna puerta o ventana se ha abierto. Por favor cerrarla."
            self.warning_web_page_main_node_publisher.publish(String(data=warning_message))
            self.warning_arduino_main_node_publisher.publish(String(data=self.warning))  
            self.get_logger().warn(f"Advertencia publicada: {warning_message}")
        elif msg.data == "Todo está bien":
            self.warning = "0"
            self.warning_web_page_main_node_publisher.publish(String(data="Todo está bien."))
            self.warning_arduino_main_node_publisher.publish(String(data=self.warning))  
            self.get_logger().info("Todo está bien.")

    def publish_modo_operacion(self):
        modo_msg = String()
        modo_msg.data = self.modo_operacion
        self.mode_operation_arduino_main_node_publisher.publish(modo_msg)

    def publish_estado_buzzer(self):
        buzzer_msg = String()
        buzzer_msg.data = self.estado_buzzer
        self.warning_arduino_main_node_publisher.publish(buzzer_msg)

    def publish_led_values(self):
        last_published_led_values = None
        while True:
            led_values_message = String()
            led_values_message.data = f"{self.LED_boca},{self.LED_orejas},{self.LED_ojos}"
            if led_values_message.data != last_published_led_values:
                self.LEDsRGB_arduino_main_node_publisher.publish(led_values_message)
                self.get_logger().info(f"Publicado en LEDsRGB_arduino: {led_values_message.data}")
                last_published_led_values = led_values_message.data  
            time.sleep(1)  

def fix_data_format(data_list):
    fixed_data = []
    for item in data_list:
        while len(item) > 0:
            if '.' in item:
                try:
                    num_end = item.find('.') + 1
                    while num_end < len(item) and item[num_end].isdigit():
                        num_end += 1
                    fixed_data.append(float(item[:num_end]))
                    item = item[num_end:]  
                except ValueError:
                    break
            else:
                break
    return fixed_data

def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
