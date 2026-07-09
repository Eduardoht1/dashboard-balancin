#include <stdint.h>
#include "TouchScreen.h"
#include <ESP32Servo.h>
#include <QuickPID.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const uint8_t PIN_YP = 33;  //amarillo ---------
const uint8_t PIN_XM = 32;  //naranja -------
const uint8_t PIN_YM = 27;  //verde
const uint8_t PIN_XP = 26;  //azul
const uint8_t PIN_SERVO1 = 14; // Frontal 14 nar
const uint8_t PIN_SERVO2 = 12; // Atrás Derecha 12 azu
const uint8_t PIN_SERVO3 = 13; // Atrás Izquierda 13 ver
const uint8_t PIN_POWER_SWITCH = 25; // Control del MOSFET

TouchScreen ts = TouchScreen(PIN_XP, PIN_YP, PIN_XM, PIN_YM, 300);
Servo s1, s2, s3;

float inputX, outputX, setpointX = 550; 
float inputY, outputY, setpointY = 550;
float tiempo;
float angS1, angS2, angS3; 

float KpY = 0.020, KiY = 0.0005, KdY = 0.0085; 
float KpX = 0.01350, KiX = 0.0005, KdX = 0.0071; 

QuickPID pidX(&inputX, &outputX, &setpointX);
QuickPID pidY(&inputY, &outputY, &setpointY);

uint32_t tiempoUltimo = 0;
const uint8_t sampleTime = 20;
uint32_t tiempoUltimaBola = 0; //Para checar falsos datos
uint32_t tiempoAhora =0;

const char* ssid = "LaptopEdu";
const char* password = "G65n72R98a02E05";

//Pagina - Configuración del Broker MQTT
const char* mqtt_server = "broker.hivemq.com";
const char* TOPIC_DATA = "delta/eduhdz/telemetria"; 
const char* TOPIC_CMD = "delta/eduhdz/comandos";   // Donde la ESP escuchará órdenes
WiFiClient espClient;
PubSubClient client(espClient);
uint32_t tiempoMqtt = 0;
const uint16_t intervaloMqtt = 50; // Enviamos a la nube cada 50ms
bool motoresHabilitados = false; 
bool controlPIDActivo = false;   

//Filtro EMA
float alpha = 0.85; // Factor de suavizado (0.95)
float emaX = 550; 
float emaY = 550;

//Correcion de error sin detectar bola
float ultimaPosXValida = 550; //Para error de no deteccion
float ultimaPosYValida = 550  ;
float posX_uso = 550; //La que se envia a Ema
float posY_uso = 550; 
bool EstadoBola = true;

//---------------------------------------------------------------------------------------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
StaticJsonDocument<400> doc;
  deserializeJson(doc, payload, length);
  
  if (doc.containsKey("motores")) motoresHabilitados = doc["motores"];
  if (doc.containsKey("control")) controlPIDActivo = doc["control"];

  if (doc.containsKey("kpx")) {
    KpX = doc["kpx"]; KiX = doc["kix"]; KdX = doc["kdx"];
    pidX.SetTunings(KpX, KiX, KdX);
  }
  if (doc.containsKey("kpy")) {
    KpY = doc["kpy"]; KiY = doc["kiy"]; KdY = doc["kdy"];
    pidY.SetTunings(KpY, KiY, KdY);
  }
  if (doc.containsKey("spx")) setpointX = doc["spx"];
  if (doc.containsKey("spy")) setpointY = doc["spy"];
  if (doc.containsKey("alpha")) alpha = doc["alpha"];
}
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    // Intentamos conectar con un ID único basado en el tiempo
    String clientId = "ESP32Delta-" + String(random(0, 999));
    if (client.connect(clientId.c_str())) {
      Serial.println("¡Conectado!");
      client.subscribe(TOPIC_CMD); // Escuchamos los botones de la web
    } else {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void setup() {
  delay(2000);
  Serial.begin(115200);
  analogReadResolution(10);

  s1.attach(PIN_SERVO1);
  s2.attach(PIN_SERVO2);
  s3.attach(PIN_SERVO3);

  //Estado inicial
  s1.write(140); 
  s2.write(140);
  s3.write(135); 
  delay(500); 

  pidX.SetTunings(KpX, KiX, KdX);
  pidY.SetTunings(KpY, KiY, KdY);
  pidX.SetMode(QuickPID::Control::automatic);
  pidY.SetMode(QuickPID::Control::automatic);
  
  //Antiwindup
  pidX.SetOutputLimits(-40, 40);
  pidY.SetOutputLimits(-40, 40);

  //Pagina
  pinMode(PIN_POWER_SWITCH, OUTPUT);
  digitalWrite(PIN_POWER_SWITCH, LOW); // Inicia apagado
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  Serial.print("IP ESP32: "); Serial.println(WiFi.localIP()); // ANOTA ESTA IP
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  client.setBufferSize(512);
}
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  tiempoAhora = millis();
  
  // Switch Motores
  if (motoresHabilitados) {
    digitalWrite(PIN_POWER_SWITCH, HIGH); 
    if (!s1.attached()) {
      s1.attach(PIN_SERVO1);
      s2.attach(PIN_SERVO2);
      s3.attach(PIN_SERVO3);
    }
  } else {
    digitalWrite(PIN_POWER_SWITCH, LOW);  
    if (s1.attached()) {
      s1.detach();
      s2.detach();
      s3.detach();
    }
  }

  if (tiempoAhora - tiempoUltimo >= sampleTime) {
    tiempoUltimo = tiempoAhora;
    TSPoint p = ts.getPoint();
    EstadoBola = true;

    if (p.z >= 0 && p.y < 950 && p.x > 100) { // Con bola normal y correcion de picos de ruido && (abs(p.x-ultimaPosXValida))<100 && (abs(p.y-ultimaPosYValida))<100
      posX_uso = p.x;                             
      posY_uso = p.y;                             
      ultimaPosXValida = p.x;                     
      ultimaPosYValida = p.y;                     
      tiempoUltimaBola = millis(); //Se va a comparar con tiempo total
      tiempo = tiempo + 0.02; 
    } else { // Checar error
      if (millis() - tiempoUltimaBola < 1000) { // Con bola, sin deteccion
        posX_uso = ultimaPosXValida;
        posY_uso = ultimaPosYValida;
      } else { // Sin Bola
        posX_uso = 0;
        posY_uso = 0;
        tiempo = 0; 
        EstadoBola = false;
      }
    }

    // ENVIAR DATOS A MQTT (Solo cada 50ms)
    if (tiempoAhora - tiempoMqtt >= intervaloMqtt) {
      tiempoMqtt = tiempoAhora;
      
      StaticJsonDocument<400> telemetria;
      telemetria["posX"] = posX_uso;
      telemetria["posY"] = posY_uso;
      telemetria["errX"] = setpointX - emaX;
      telemetria["errY"] = setpointY - emaY;
      telemetria["kpx"] = KpX; telemetria["kix"] = KiX; telemetria["kdx"] = KdX;
      telemetria["kpy"] = KpY; telemetria["kiy"] = KiY; telemetria["kdy"] = KdY;
      telemetria["spx"] = setpointX; 
      telemetria["spy"] = setpointY; 
      telemetria["s1"] = angS1; telemetria["s2"] = angS2; telemetria["s3"] = angS3;
      telemetria["m_on"] = motoresHabilitados;
      telemetria["c_on"] = controlPIDActivo;
      telemetria["alpha"] = alpha;

      char buffer[400];
      serializeJson(telemetria, buffer);
      client.publish(TOPIC_DATA, buffer); 
    }
    
    // Filtro EMA
    if (EstadoBola) { //Con Bola                       
      emaX = (posX_uso * alpha) + (emaX * (1.0 - alpha)); 
      emaY = (posY_uso * alpha) + (emaY * (1.0 - alpha)); 
    } else { //Sin Bola                                 
      emaX = 550;                                
      emaY = 550;                                 
    }

    inputX = emaX; 
    inputY = emaY;
      
    if (EstadoBola) {                          
      Serial.print("X: ");
      Serial.print(posX_uso);                       
      Serial.print(" | Y: ");
      Serial.print(posY_uso);                       
      Serial.print(" | Presion: ");
      Serial.print(p.z); 
      Serial.print(" | T (s): ");
      Serial.println(tiempo);
    }                                             
    
    if (controlPIDActivo && motoresHabilitados && EstadoBola) {
        pidX.Compute();
        pidY.Compute();
        actualizarPlataforma(outputX, outputY);
    } else { 
        actualizarPlataforma(0, 0); 
    }
  }
}
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void actualizarPlataforma(float outX, float outY) {
  const uint8_t HOME = 140; 
  const float FACTOR_Y = sqrt(3) / 2.0;

  float val1 = HOME - outX;
  float val2 = HOME + (0.5 * outX) + (FACTOR_Y * outY);
  float val3 = HOME - 5 + (0.5 * outX) - (FACTOR_Y * outY);

  //Pruebas y
  //float val1 = HOME;
  //float val2 = HOME  + (FACTOR_Y * outY);
  //float val3 = HOME  - (FACTOR_Y * outY);

  //Pruebas x
  //float val1 = HOME - outX;
  //float val2 = HOME + (0.5 * outX);
  //float val3 = HOME - 5 + (0.5 * outX);

  angS1 = constrain(val1, 120, 160);
  angS2 = constrain(val2, 120, 160);
  angS3 = constrain(val3, 115, 155);

  s1.write(angS1); 
  s2.write(angS2);
  s3.write(angS3);
}
//---------------------------------------------------------------------------------------------------------------------
