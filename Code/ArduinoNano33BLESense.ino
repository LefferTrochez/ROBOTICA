#include <Servo.h>  
#include "Arduino_BMI270_BMM150.h"  
#include <Arduino_APDS9960.h> 
#include <PDM.h>
#include <Arduino_HS300x.h>
#include <Adafruit_AMG88xx.h> 
#define NUM_VALORES 10  
#define LEDR 22          
#define LEDG 23          
#define LEDB 24 
#define SERVO_PIN 13    
#define SERVO_TIL_1 12   
#define SERVO_TIL_2 11   
#define RELAY_BOCA_PIN 10  
#define RELAY_OREJAS_PIN 4 
#define LEDR_OJOS 7       
#define LEDG_OJOS 6       
#define LEDB_OJOS 5       
#define BUZZER_PIN A1    
#define MODO_SLEEP_PIN A2 

Adafruit_AMG88xx amg; 
String servo_pan = "00";
String servo_til = "0000";  
String control_ojos = "000";
Servo servo;         
Servo servo_til_1;    
Servo servo_til_2;   

float valoresVoltaje[NUM_VALORES]; 
int indiceVoltaje = 0;         
bool EncenderLEDsPotencia = false;  
const unsigned long TIEMPO_MOVIMIENTO_SERVO = 100;  
const unsigned long TIEMPO_MOVIMIENTO_SERVO_LARGO = 500; 
float ax, ay, az;
float gx, gy, gz;
float roll = 0;
float pitch = 0;
float yaw = 0;
float initialYaw = 0, initialPitch = 0, initialRoll = 0;
unsigned long previousMillis = 0;
unsigned long startTimeServoPan = 0;  
bool servoPanActivo = false;       
int angulo_til_1 = 60;  
int angulo_til_2 = 100; 
const int MIN_TIL1 = 20;
const int MAX_TIL1 = 120;
const int MIN_TIL2 = 50;
const int MAX_TIL2 = 155;
const int RESET_TIL1 = 60;
const int RESET_TIL2 = 100;
const int RESOLUCION = 6; 
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
bool gesto_presente = false; 
int ultimoAmbientLight = 0;
int ultimoProximidad = 0; 
int stop_buzzer_gestos = 0;  
unsigned long tiempoApagadoBuzzer = 0;
const unsigned long tiempoEsperarRuido = 5000; 
float old_decibels = 0.0;
const int pinDivisorVoltaje = A3;
float vout = 0.0;
float vin = 0.0;
float R1 = 10000.0;
float R2 = 5600.0;
int analogValue = 0;
const float referenceVoltage = 3.3;
bool cambioModoDesdeGesto = false;
short sampleBuffer[256];
volatile int samplesRead;
float rmsValue;
float decibels = 0.0;
float old_temp = 0.0;  
float temperature = 0.0;  
int mq135Pin = A0;  
int mq135Value = 0;
const int pinIR = 0;
const int pinSensorFuego = A7;
int valorSensorFuego = 0;
float voltajeSensorFuego = 0.0;
const float referenciaVoltajeSensorFuego = 3.4; 
const float threshold_earthquake = 20.0;
int threshold_luz = 140;  
bool pinA6Controlado = false; 
int ambientLight = 0;  
int modo_operacion = 1;  
int estado_buzzer = 0;  
int control_boca = 0;     
int control_orejas = 0; 
int resolucion_pasos = 2; 
unsigned long previousMillis_ojos = 0; 
const long interval_ojos = 500;  
int color_actual_ojos = 0;     
bool secuencia_activa_ojos = false;  
const int colores[][3] = {
  {255, 0, 0},        
  {0, 255, 0},     
  {0, 0, 255},   
  {255, 255, 0},      
  {0, 255, 255},     
  {255, 0, 255},     
  {255, 255, 255},    
  {128, 0, 128},      
  {128, 128, 0},      
  {0, 128, 128}       
};
const int numero_colores = sizeof(colores) / sizeof(colores[0]); 
bool nuevo_mensaje_recibido = false;  

void setup() {

  Serial.begin(115200);
  pinMode(A6, OUTPUT);
  pinMode(pinIR, OUTPUT);
  pinMode(mq135Pin, INPUT);
  pinMode(LEDR, OUTPUT);       
  pinMode(LEDG, OUTPUT);      
  pinMode(LEDB, OUTPUT);  
  pinMode(A6, OUTPUT);  
  digitalWrite(A6, LOW);
  digitalWrite(pinIR, LOW);
  pinMode(RELAY_BOCA_PIN, OUTPUT);  
  pinMode(RELAY_OREJAS_PIN, OUTPUT);  
  pinMode(LEDR_OJOS, OUTPUT);         
  pinMode(LEDG_OJOS, OUTPUT);        
  pinMode(LEDB_OJOS, OUTPUT);        
  pinMode(BUZZER_PIN, OUTPUT);   
  pinMode(MODO_SLEEP_PIN, OUTPUT);
  servo.attach(SERVO_PIN);      
  servo_til_1.attach(SERVO_TIL_1); 
  servo_til_2.attach(SERVO_TIL_2); 
  actualizarModoOperacion(modo_operacion);
  inicializarSensorTerremoto();  

  if (!APDS.begin()) {
    while (1);  
  }

  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, 16000)) {
    while (1); 
  }

  bool status = amg.begin();
  if (!status) {
    while (1);
  }

  if (!HS300x.begin()) {
    while (1);  
  }
}

void inicializarSensorTerremoto() {
  if (!IMU.begin()) {
    while (1); 
  }
}

void leerCamaraTermica(float *pixels) {
  amg.readPixels(pixels);
}

int leerMQ135() {
  mq135Value = analogRead(mq135Pin);
  return mq135Value;
}

float leerSensorFuego() {
  valorSensorFuego = analogRead(pinSensorFuego);
  voltajeSensorFuego = (valorSensorFuego * referenciaVoltajeSensorFuego) / 1023.0;
  return voltajeSensorFuego;
}

bool motoresDetenidos = false;

int leerSensorTerremoto() {
    int earthquakeStatus = 0;
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);
        calculateOrientation();
        if (servo_pan == "00" && servo_til == "0000") {
            if (!motoresDetenidos) {
                resetOrientation();  
                motoresDetenidos = true;
            }
            float adjustedYaw = yaw - initialYaw;
            float adjustedPitch = pitch - initialPitch;
            float adjustedRoll = roll - initialRoll;
            adjustedYaw = normalizeAngle(adjustedYaw);
            adjustedPitch = normalizeAngle(adjustedPitch);
            adjustedRoll = normalizeAngle(adjustedRoll);
            if (abs(adjustedYaw) > threshold_earthquake || 
                abs(adjustedPitch) > threshold_earthquake || 
                abs(adjustedRoll) > threshold_earthquake) {
                earthquakeStatus = 1;
            }
        } else {
            motoresDetenidos = false;  
            earthquakeStatus = 0;
        }
    }
    return earthquakeStatus;
}

void calculateOrientation() {
  roll = atan2(ay, az) * 180 / PI;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;
  float elapsedTime = (millis() - previousMillis) / 1000.0; 
  previousMillis = millis();
  yaw += gz * elapsedTime; 
}

void resetOrientation() {
  initialYaw = yaw;
  initialPitch = pitch;
  initialRoll = roll;
}

float normalizeAngle(float angle) {
  while (angle > 90) angle -= 180;
  while (angle < -90) angle += 180;
  return angle;
}

int leerLuzAmbiente() {
  if (APDS.colorAvailable()) {  
    int r, g, b;
    APDS.readColor(r, g, b, ambientLight); 
    return ambientLight;
  }
  return -1;  
}

int leerSensorProximidad() {
  if (APDS.proximityAvailable()) {
    int proximidad = APDS.readProximity(); 
    if (gesto_presente && proximidad < 50) {
      return 255;
    }
    return proximidad;  
  }
  return -1;  
}

void enviarDatosPorSerial(int modo_operacion, int earthquakeStatus, int proximity, float decibels, float temperature, int ambientLight, float mq135Value, float sensorFuego, float *pixels) {
    Serial.print(modo_operacion);
    Serial.print(",");  
    if (stop_buzzer_gestos == 2) {
        Serial.print(stop_buzzer_gestos);  
        stop_buzzer_gestos = 0;  
    } else {
        Serial.print(0); 
    }
    Serial.print(",");
    Serial.print(earthquakeStatus);
    Serial.print(",");
    if (proximity >= 0) {
        ultimoProximidad = proximity;  
    }
    Serial.print(ultimoProximidad); 
    Serial.print(",");
    Serial.print(decibels);
    Serial.print(",");
    Serial.print(temperature);
    Serial.print(",");
    if (ambientLight != -1) {
        ultimoAmbientLight = ambientLight;  
    }
    Serial.print(ultimoAmbientLight); 
    Serial.print(",");
    Serial.print(mq135Value);
    Serial.print(",");
    Serial.print(sensorFuego); 
    Serial.print(",");
    for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
        Serial.print(pixels[i]);
        if (i < AMG88xx_PIXEL_ARRAY_SIZE - 1) { 
            Serial.print(", ");
        }
    }
    Serial.println();
}

void leerVoltaje() {
  analogValue = analogRead(pinDivisorVoltaje);
  vout = (analogValue * referenceVoltage) / 1023.0;
  vin = vout / (R2 / (R1 + R2));
  valoresVoltaje[indiceVoltaje] = vin;
  indiceVoltaje = (indiceVoltaje + 1) % NUM_VALORES; 
  float minValor = valoresVoltaje[0];
  float maxValor = valoresVoltaje[0];
  for (int i = 1; i < NUM_VALORES; i++) {
    if (valoresVoltaje[i] < minValor) minValor = valoresVoltaje[i];
    if (valoresVoltaje[i] > maxValor) maxValor = valoresVoltaje[i];
  }
  if ((maxValor - minValor) > 0.2) {
    EncenderLEDsPotencia = false;  
  } else {
    EncenderLEDsPotencia = true;  
  }
  if (modo_operacion == 0) {  
    setColor(0, 0, 255); 
  } else if (EncenderLEDsPotencia) {
    setColor(255, 0, 0); 
  } else {
    setColor(0, 255, 0); 
  }
}

void actualizarModoOperacion(int nuevo_modo) {
  if (nuevo_modo == 1) {
    setColor(0, 255, 0);  
    digitalWrite(MODO_SLEEP_PIN, LOW);  
  } else if (nuevo_modo == 2) {
    setColor(255, 0, 0);  
    digitalWrite(MODO_SLEEP_PIN, LOW); 
  } else {
    setColor(0, 0, 255); 
    digitalWrite(MODO_SLEEP_PIN, HIGH); 
  }
}

void loop() {

  if (servoPanActivo && ((servo_pan == "01" || servo_pan == "10") && (millis() - startTimeServoPan >= TIEMPO_MOVIMIENTO_SERVO))) {
      servo.write(90);  
      servoPanActivo = false;  
  } else if (servoPanActivo && ((servo_pan == "02" || servo_pan == "20") && (millis() - startTimeServoPan >= TIEMPO_MOVIMIENTO_SERVO_LARGO))) {
      servo.write(90); 
      servoPanActivo = false; 
  }
  leerVoltaje();
  if (modo_operacion != 2) {
    if (Serial.available() > 0) {
      String mensaje = Serial.readStringUntil('\n');
      int separador1 = mensaje.indexOf(',');
      int separador2 = mensaje.indexOf(',', separador1 + 1);
      int separador3 = mensaje.indexOf(',', separador2 + 1);
      int separador4 = mensaje.indexOf(',', separador3 + 1);
      int separador5 = mensaje.indexOf(',', separador4 + 1);
      int separador6 = mensaje.indexOf(',', separador5 + 1);
      String mensaje_modo = mensaje.substring(0, separador1);
      String mensaje_buzzer = mensaje.substring(separador1 + 1, separador2);
      servo_pan = mensaje.substring(separador2 + 1, separador3); 
      servo_til = mensaje.substring(separador3 + 1, separador4); 
      control_boca = mensaje.substring(separador4 + 1, separador5).toInt();  
      control_orejas = mensaje.substring(separador5 + 1, separador6).toInt();
      control_ojos = mensaje.substring(separador6 + 1); 
      modo_operacion = mensaje_modo.toInt();
      estado_buzzer = mensaje_buzzer.toInt();
      if (servo_pan == "00" || servo_pan == "01" || servo_pan == "10") {
          controlarServoPan(servo_pan);
      } else if (servo_pan == "00" || servo_pan == "20" || servo_pan == "02") {
          controlarServoPanExtendido(servo_pan); 
      }
      actualizarModoOperacion(modo_operacion);
      controlarBuzzer(estado_buzzer);
      controlarServosTil(servo_til);
      controlarServosTilAltaResolucion(servo_til);
      controlarRelayBoca(control_boca);
      controlarRelayOrejas(control_orejas);
      controlarLEDOjos(control_ojos);
      nuevo_mensaje_recibido = true;
      } else if (APDS.colorAvailable()) {
          if (ambientLight < 16) { 
              digitalWrite(pinIR, HIGH);    
              if (EncenderLEDsPotencia) { 
                  digitalWrite(A6, LOW); 
              } else {
                  digitalWrite(A6, LOW);  
              }
          } else {
              digitalWrite(pinIR, LOW);    
              digitalWrite(A6, LOW);      
          }
      } else {
          if (ambientLight < 16) { 
              digitalWrite(pinIR, HIGH);    
              if (EncenderLEDsPotencia) {  
                  digitalWrite(A6, LOW);  
              } else {
                  digitalWrite(A6, LOW);  
              }
          } else {
              digitalWrite(pinIR, LOW);    
              digitalWrite(A6, LOW);      
          }
      }

if (APDS.gestureAvailable()) {
    int gesture = APDS.readGesture(); 
    gesto_presente = true;
    if (gesture == GESTURE_RIGHT) {
        modo_operacion = 0;  
        actualizarModoOperacion(modo_operacion);
        cambioModoDesdeGesto = true;
    } 
    else if (gesture == GESTURE_LEFT) {
        modo_operacion = 1; 
        actualizarModoOperacion(modo_operacion);
        cambioModoDesdeGesto = true;
    } 
    else if (gesture == GESTURE_UP || gesture == GESTURE_DOWN) {
        digitalWrite(BUZZER_PIN, LOW); 
        estado_buzzer = 0;  
        stop_buzzer_gestos = 2;  
    }
} else {
    gesto_presente = false;  
}
}
  manejarSecuenciaOjos();
  int earthquakeStatus = leerSensorTerremoto(); 
  int proximity = leerSensorProximidad();
  int ambientLight = leerLuzAmbiente(); 
  int mq135Value = leerMQ135();
  float sensorFuego = leerSensorFuego();
  verificarPanTerremoto(servo_pan, earthquakeStatus);
  leerCamaraTermica(pixels);
  if (samplesRead) {
    rmsValue = calculateRMS(sampleBuffer, samplesRead);
    decibels = 20 * log10(rmsValue);  
    samplesRead = 0;  
  }
  bool buzzerActivo = digitalRead(BUZZER_PIN) == HIGH;  
  verificarBuzzerRuido(decibels, old_decibels, buzzerActivo);
  temperature = HS300x.readTemperature();
  if (abs(old_temp - temperature) >= 0.5) {  
    old_temp = temperature;  
  }
  enviarDatosPorSerial(modo_operacion, earthquakeStatus, proximity, decibels, temperature, ambientLight, mq135Value, sensorFuego, pixels);
}

void verificarPanTerremoto(String pan, int &earthquakeStatus) {
  if (pan == "11" && earthquakeStatus == 1) {
    earthquakeStatus = 0;
  }
}

void manejarSecuenciaOjos() {
    unsigned long currentMillis = millis();
    if (secuencia_activa_ojos && (currentMillis - previousMillis_ojos >= interval_ojos)) {
        previousMillis_ojos = currentMillis;
        secuenciaColoresOjos();
    }
}

void setColor(int red, int green, int blue) {
  digitalWrite(LEDR, red == 255 ? LOW : HIGH);
  digitalWrite(LEDG, green == 255 ? LOW : HIGH);
  digitalWrite(LEDB, blue == 255 ? LOW : HIGH);
}

void controlarBuzzer(int estado) {
  digitalWrite(BUZZER_PIN, estado == 1 ? HIGH : LOW);
}

void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2; 
}

float calculateRMS(short *buffer, int length) {
  long sumOfSquares = 0;
  for (int i = 0; i < length; i++) {
    sumOfSquares += (buffer[i] * buffer[i]);
  }
  float meanSquare = (float)sumOfSquares / length;
  return sqrt(meanSquare);
}

void controlarServoPan(String pan) {
    if (pan == "00") {
        servo.write(90);  
        servoPanActivo = false;  
    } else if (!servoPanActivo) {
        if (pan == "01") {
            servo.write(180); 
        } else if (pan == "10") {
            servo.write(0); 
        }
        startTimeServoPan = millis(); 
        servoPanActivo = true;
    }
}

void controlarServoPanExtendido(String pan) {
    if (pan == "00") {
        servo.write(90);  
        servoPanActivo = false; 
    } else if (!servoPanActivo) {
        if (pan == "02") {
            servo.write(180); 
        } else if (pan == "20") {
            servo.write(0); 
        }
        startTimeServoPan = millis(); 
        servoPanActivo = true; 
    }
}

void controlarServosTil(String til) {
  if (servo_til == "0001") {  
    if (angulo_til_1 - RESOLUCION >= MIN_TIL1) angulo_til_1 -= RESOLUCION;
    if (angulo_til_2 + RESOLUCION <= MAX_TIL2) angulo_til_2 += RESOLUCION;
  } else if (servo_til == "0010") {  
    if (angulo_til_1 + RESOLUCION <= MAX_TIL1) angulo_til_1 += RESOLUCION;
    if (angulo_til_2 - RESOLUCION >= MIN_TIL2) angulo_til_2 -= RESOLUCION;
  } else if (servo_til == "0011") { 
    if (angulo_til_1 < RESET_TIL1) angulo_til_1 = min(angulo_til_1 + RESOLUCION, RESET_TIL1);
    else if (angulo_til_1 > RESET_TIL1) angulo_til_1 = max(angulo_til_1 - RESOLUCION, RESET_TIL1);
    if (angulo_til_2 < RESET_TIL2) angulo_til_2 = min(angulo_til_2 + RESOLUCION, RESET_TIL2);
    else if (angulo_til_2 > RESET_TIL2) angulo_til_2 = max(angulo_til_2 - RESOLUCION, RESET_TIL2);
  } else if (servo_til == "0100") {  
    if (angulo_til_1 - RESOLUCION >= MIN_TIL1) angulo_til_1 -= RESOLUCION;
    if (angulo_til_2 - RESOLUCION >= MIN_TIL2) angulo_til_2 -= RESOLUCION;
  } else if (servo_til == "0110") {  
    if (angulo_til_1 + RESOLUCION <= MAX_TIL1) angulo_til_1 += RESOLUCION;
    if (angulo_til_2 + RESOLUCION <= MAX_TIL2) angulo_til_2 += RESOLUCION;
  }
  servo_til_1.write(angulo_til_1);
  servo_til_2.write(angulo_til_2);
}

void controlarServosTilAltaResolucion(String til) {
  int target_til_1 = angulo_til_1;
  int target_til_2 = angulo_til_2;
  if (til == "0101") {  
    target_til_1 = MIN_TIL1;  
    target_til_2 = MAX_TIL2;  
  } else if (til == "0111") {  
    target_til_1 = MAX_TIL1;  
    target_til_2 = MIN_TIL2;  
  } else if (til == "1111") {  
    target_til_1 = RESET_TIL1;  
    target_til_2 = RESET_TIL2;  
  }
  while (angulo_til_1 != target_til_1 || angulo_til_2 != target_til_2) {
    if (angulo_til_1 < target_til_1) {
      angulo_til_1++;  
    } else if (angulo_til_1 > target_til_1) {
      angulo_til_1--;  
    }
    if (angulo_til_2 < target_til_2) {
      angulo_til_2++;  
    } else if (angulo_til_2 > target_til_2) {
      angulo_til_2--; 
    }
    servo_til_1.write(angulo_til_1);
    servo_til_2.write(angulo_til_2);
    delay(10);  
  }
}

void verificarBuzzerRuido(float &decibels, float &old_decibels, bool buzzerActivo) {
    static unsigned long tiempoApagadoBuzzer = 0;  
    unsigned long currentMillis = millis(); 
    if (buzzerActivo && decibels > 50) {
        tiempoApagadoBuzzer = 0; 
        decibels = random(5, 26); 
    } else {
        if (tiempoApagadoBuzzer == 0) {
            tiempoApagadoBuzzer = currentMillis;
        }
        if (currentMillis - tiempoApagadoBuzzer < tiempoEsperarRuido) {
            decibels = random(5, 26); 
        } else {
            old_decibels = decibels;
        }
    }
}

void controlarRelayBoca(int control) {
  digitalWrite(RELAY_BOCA_PIN, control == 1 ? HIGH : LOW);
}

void controlarRelayOrejas(int control) {
  digitalWrite(RELAY_OREJAS_PIN, control == 1 ? HIGH : LOW);
}

void controlarLEDOjos(String control) {
  if (control == "000") {
    setColorOjos(255, 0, 0); 
    secuencia_activa_ojos = false;
  } else if (control == "001") {
    setColorOjos(0, 255, 0);  
    secuencia_activa_ojos = false;
  } else if (control == "010") {
    setColorOjos(0, 0, 255); 
    secuencia_activa_ojos = false;
  } else if (control == "011") {
    setColorOjos(255, 255, 0);  
    secuencia_activa_ojos = false;
  } else if (control == "100") {
    setColorOjos(0, 255, 255);  
    secuencia_activa_ojos = false;
  } else if (control == "101") {
    setColorOjos(255, 0, 255); 
    secuencia_activa_ojos = false;
  } else if (control == "110") {
    setColorOjos(255, 255, 255);  
    secuencia_activa_ojos = false;
  } else if (control == "111") {
    secuencia_activa_ojos = true;  
    color_actual_ojos = 0;        
  } 
}

void setColorOjos(int red, int green, int blue) {
  analogWrite(LEDR_OJOS, red);
  analogWrite(LEDG_OJOS, green);
  analogWrite(LEDB_OJOS, blue);
}

void secuenciaColoresOjos() {
  setColorOjos(colores[color_actual_ojos][0], colores[color_actual_ojos][1], colores[color_actual_ojos][2]);
  color_actual_ojos = (color_actual_ojos + 1) % numero_colores;
}
