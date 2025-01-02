#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import time
import numpy as np
import threading

class IRCutController:
    CHIP_I2C_ADDR = 0x0C
    OPT_IRCUT = 0x1000 | 0x05 
    OPT_ZOOM = 0x1000 | 0x02 
    OPT_FOCUS = 0x1000 | 0x01
    opts = {
        OPT_IRCUT: {"REG_ADDR": 0x0C, "MIN_VALUE": 0x00, "MAX_VALUE": 0x01, "RESET_ADDR": None},
        OPT_ZOOM: {"REG_ADDR": 0x00, "MIN_VALUE": 215, "MAX_VALUE": 2100, "RESET_ADDR": None},
        OPT_FOCUS: {"REG_ADDR": 0x01, "MIN_VALUE": 0, "MAX_VALUE": 1830, "RESET_ADDR": None},
    }
    def __init__(self, bus):
        try:
            import smbus
            self.bus = smbus.SMBus(bus)
        except Exception as e:
            print(f"Error al inicializar SMBus: {e}")
            raise

    def read(self, chip_addr, reg_addr):
        value = self.bus.read_word_data(chip_addr, reg_addr)
        return ((value & 0x00FF) << 8) | ((value & 0xFF00) >> 8)

    def write(self, chip_addr, reg_addr, value):
        value = max(0, value)
        value = ((value & 0x00FF) << 8) | ((value & 0xFF00) >> 8)
        self.bus.write_word_data(chip_addr, reg_addr, value)

    def is_busy(self):
        busy_reg_addr = 0x04  
        return self.read(self.CHIP_I2C_ADDR, busy_reg_addr) != 0

    def waiting_for_free(self):
        count = 0
        while self.is_busy() and count < 500:
            count += 1
            time.sleep(0.01)

    def set(self, opt, value):
        self.waiting_for_free()
        info = self.opts[opt]
        value = min(max(value, info["MIN_VALUE"]), info["MAX_VALUE"])
        self.write(self.CHIP_I2C_ADDR, info["REG_ADDR"], value)

    def toggle_ircut(self, state):
        self.set(self.OPT_IRCUT, 0x01 if state else 0x00)

class CameraZoomFocusNode(Node):
    def __init__(self):
        super().__init__("camera_zoom_focus_IR")
        self.get_logger().info("Node camera_zoom_focus initialized")
        self.coeficientes = [-1.225e-10, 9.413e-07, -0.002935, 4.41, -821.8]
        self.zoom = 215
        self.auto_zoom = False 
        try:
            self.ircut_controller = IRCutController(bus=1)
            self.get_logger().info("IRCut controller initialized")
        except Exception as e:
            self.get_logger().error(f"Error initializing IRCut controller: {e}")
            self.ircut_controller = None
        self.coordinates_subscription = self.create_subscription(Float32MultiArray,'object_coordinates',self.coordinates_callback,10)
        self.ambient_light_subscription = self.create_subscription(Bool,'ambient_light_sensor',self.ambient_light_callback,10)
        self.ircut_controller.set(IRCutController.OPT_ZOOM, self.zoom)
        initial_focus = self.calcular_focus(self.zoom)
        self.ircut_controller.set(IRCutController.OPT_FOCUS, initial_focus)
        self.get_logger().info(f"Initialized with zoom: {self.zoom}, focus: {initial_focus}")
        auto_zoom_thread = threading.Thread(target=self.auto_zoom_logic, daemon=True)
        auto_zoom_thread.start()
        user_input_thread = threading.Thread(target=self.handle_user_input, daemon=True)
        user_input_thread.start()

    def ambient_light_callback(self, msg):
        if self.ircut_controller:
            self.ircut_controller.toggle_ircut(msg.data)
            state = "activated" if msg.data else "deactivated"

    def calcular_focus(self, zoom):
        focus = np.polyval(self.coeficientes, zoom)
        return int(round(focus))

    def coordinates_callback(self, msg):
        if len(msg.data) == 4:
            x1, y1, x2, y2 = msg.data
            box_height = abs(y2 - y1)
            frame_height = 480  
            if self.auto_zoom and box_height < 0.98 * frame_height:
                self.zoom = min(self.zoom + 50, 2100) 
                focus = self.calcular_focus(self.zoom)
                self.ircut_controller.set(IRCutController.OPT_ZOOM, self.zoom)
                self.ircut_controller.set(IRCutController.OPT_FOCUS, focus)

    def auto_zoom_logic(self):
        while True:
            if not self.auto_zoom and self.zoom != 215:
                self.zoom = 215
                focus = self.calcular_focus(self.zoom)
                try:
                    self.ircut_controller.set(IRCutController.OPT_ZOOM, self.zoom)
                    self.ircut_controller.set(IRCutController.OPT_FOCUS, focus)
                    self.get_logger().info(f"Resetting zoom to {self.zoom}, focus to {focus}")
                except Exception as e:
                    self.get_logger().error(f"Error resetting zoom or focus: {e}")
            time.sleep(0.1) 

    def handle_user_input(self):
        while True:
            user_input = input("¿Activar zoom automático? (True/False): ").strip().lower()
            if user_input == "true":
                self.auto_zoom = True
                self.get_logger().info("Auto zoom activated by user")
            elif user_input == "false":
                self.auto_zoom = False
                self.get_logger().info("Auto zoom deactivated by user")
            else:
                print("Entrada no válida. Por favor, escribe 'True' o 'False'.")

def main(args=None):
    rclpy.init(args=args)
    node = CameraZoomFocusNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()