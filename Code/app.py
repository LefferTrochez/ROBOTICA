from flask import Flask, render_template, Response
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread
import time

app = Flask(__name__)
latest_frame = None
warning_message = "Todo está bien."
warning_active = False 
last_stop_time = 0 
last_stop_click_time = 0

class WebPageNode(Node):
    def __init__(self):
        super().__init__("web_page_node")
        self.frame_web_page_subscription = self.create_subscription(String, 'frame_web_page', self.frame_callback, 10)
        self.warning_subscription = self.create_subscription(String, 'warning_web_page', self.warning_callback, 10)
        self.sensor_subscription = self.create_subscription(String, 'sensor_values_web_page', self.sensor_callback, 10)
        self.mode_operation_publisher = self.create_publisher(String, 'mode_operation_web_page', 10)
        self.stop_warning_publisher = self.create_publisher(String, 'stop_warning_web_page', 10)
        self.motion_control_publisher = self.create_publisher(String, 'motion_control_web_page', 10)
        self.motion_pan_publisher = self.create_publisher(String, 'motion_pan_web_page', 10)
        self.motion_til_publisher = self.create_publisher(String, 'motion_til_web_page', 10)
        self.sensor_values = {
            'noise_level': '--',
            'ambient_temp': '--',
            'body_temp': '--',
            'gas_level': '--'
        }
        self.last_pan_value = None
        self.last_til_value = None
        self.last_mode_value = None
        self.last_buzzer_value = None

    def frame_callback(self, msg):
        global latest_frame
        frame_data = bytes.fromhex(msg.data)
        latest_frame = frame_data

    def warning_callback(self, msg):
        global warning_message, warning_active
        warning_message = msg.data 
        warning_active = msg.data != "Todo está bien." 
        self.get_logger().info(f"Advertencia recibida: {msg.data}")
        if warning_active:
            self.get_logger().info("[DEBUG] Advertencia activada: Se enviará vibración y sonido.")
        else:
            self.get_logger().info("[DEBUG] No hay advertencia activa.")

    def sensor_callback(self, msg):
        try:
            values = msg.data.split(',')
            self.sensor_values = {
                'noise_level': int(float(values[0])), 
                'ambient_temp': int(float(values[1])),
                'body_temp': int(float(values[2])), 
                'gas_level': int(float(values[3]))
            }
            self.get_logger().info(f"[DEBUG] Valores recibidos en sensor_values_web_page: {self.sensor_values}")
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Error al procesar los datos del tópico sensor_values_web_page: {msg.data}. {e}")

    def publish_mode_operation(self, mode):
        mode_value = '1' if mode else '0' 
        self.last_mode_value = mode_value 
        msg = String()
        msg.data = mode_value
        self.mode_operation_publisher.publish(msg) 
        self.get_logger().info(f"[DEBUG] Publicado modo de operación: {mode_value}")

    def publish_buzzer_off(self):
        msg = String()
        msg.data = '3'
        self.stop_warning_publisher.publish(msg)  
        self.get_logger().info("[DEBUG] Publicado buzzer: Off (3)")

    def publish_motion(self, direction):
        msg = String()
        msg.data = direction
        self.motion_control_publisher.publish(msg)
        self.get_logger().info(f"Movimiento enviado: {direction}")

    def publish_motion_pan(self, value):
        msg = String()
        msg.data = value
        self.motion_pan_publisher.publish(msg)
        self.get_logger().info(f"[DEBUG] Publicado motion_pan_web_page: {value}")

    def publish_motion_til(self, value):
        msg = String()
        msg.data = value
        self.motion_til_publisher.publish(msg)
        self.get_logger().info(f"[DEBUG] Publicado motion_til_web_page: {value}")

def start_ros_node():
    rclpy.init()
    global ros_node
    ros_node = WebPageNode()
    rclpy.spin(ros_node)
    rclpy.shutdown()

Thread(target=start_ros_node, daemon=True).start()


@app.route("/")
def index():
    global warning_message, warning_active
    sensor_values = ros_node.sensor_values if 'ros_node' in globals() else {
        'noise_level': '--',
        'ambient_temp': '--',
        'body_temp': '--',
        'gas_level': '--'
    }
    return render_template(
        "index.html",
        warning_message=warning_message,
        warning_active=warning_active,
        sensor_values=sensor_values
    )

def generate_video_feed():
    global latest_frame
    while True:
        if latest_frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n')
        else:
            time.sleep(0.01)

@app.route("/video_feed")
def video_feed():
    return Response(generate_video_feed(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/set_mode_on")
def set_mode_on():
    ros_node.publish_mode_operation(True)
    print("[DEBUG] Botón Active presionado (modo de operación = '1')")
    return "Modo Operacion = On"

@app.route("/set_mode_off")
def set_mode_off():
    ros_node.publish_mode_operation(False) 
    ros_node.publish_motion_pan("00") 
    ros_node.publish_motion_til("0000")
    print("[DEBUG] Botón Sleep presionado (motion_pan_web_page: 00, motion_til_web_page: 0000)")
    return "Modo Operacion = Off"

@app.route("/buzzer_off")
def buzzer_off():
    ros_node.publish_buzzer_off()
    ros_node.publish_motion_pan("00") 
    ros_node.publish_motion_til("0000") 
    print("[DEBUG] Botón Silent presionado (motion_pan_web_page: 00, motion_til_web_page: 0000)")
    return "Buzzer = Off"

@app.route("/move_up")
def move_up():
    ros_node.publish_motion_til("0101") 
    print("[DEBUG] Flecha hacia arriba presionada (motion_til_web_page: 0101)")
    return "Movimiento: Arriba"

@app.route("/move_down")
def move_down():
    ros_node.publish_motion_til("0111") 
    print("[DEBUG] Flecha hacia abajo presionada (motion_til_web_page: 0111)")
    return "Movimiento: Abajo"

@app.route("/move_left")
def move_left():
    ros_node.publish_motion_pan("20") 
    print("[DEBUG] Flecha hacia la izquierda presionada (motion_pan_web_page: 10)")
    return "Movimiento: Izquierda"

@app.route("/move_right")
def move_right():
    ros_node.publish_motion_pan("02") 
    print("[DEBUG] Flecha hacia la derecha presionada (motion_pan_web_page: 01)")
    return "Movimiento: Derecha"

@app.route("/move_stop")
def move_stop():
    global last_stop_click_time
    current_time = time.time() 
    if current_time - last_stop_click_time < 1.0:
        ros_node.publish_motion_pan("00")
        ros_node.publish_motion_til("1111")
        print("[DEBUG] Doble clic en Stop (motion_pan_web_page: 00, motion_til_web_page: 011)")
    else:
        ros_node.publish_motion_pan("00")
        ros_node.publish_motion_til("0000")
        print("[DEBUG] Clic único en Stop (motion_pan_web_page: 00, motion_til_web_page: 0000)")
    last_stop_click_time = current_time
    return "Movimiento: Detenido"

@app.route("/get_sensor_data", methods=["GET"])
def get_sensor_data():
    global warning_message
    try:
        data = {
            "warning_message": warning_message,
            "warning_active": warning_active, 
            "sensor_values": ros_node.sensor_values if 'ros_node' in globals() else {
                "noise_level": "--",
                "ambient_temp": "--",
                "body_temp": "--",
                "gas_level": "--"
            }
        }
        app.logger.info(f"Datos enviados al frontend: {data}")
        return data 
    except Exception as e:
        app.logger.error(f"Error al enviar datos al frontend: {e}")
        return {"error": "No se pudieron obtener los datos"}, 500

@app.route("/set_mode_active")
def set_mode_active():
    ros_node.publish_mode_operation(True) 
    return "Modo Operación: Activo (1)"

@app.route("/set_mode_sleep")
def set_mode_sleep():
    ros_node.publish_mode_operation(False)
    return "Modo Operación: Sleep (0)"

@app.route("/set_buzzer_off")
def set_buzzer_off():
    ros_node.publish_buzzer_off()  
    return "Buzzer: Off (3)"

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
