#include <stdint.h>
#include "TouchScreen.h"
#include <ESP32Servo.h>
#include <QuickPID.h>

const uint8_t PIN_YP = 33;  // Análogo 
const uint8_t PIN_XM = 32;  // Análogo 
const uint8_t PIN_YM = 27;  //
const uint8_t PIN_XP = 26;  //

const uint8_t PIN_SERVO1 = 14; // Frontal 14 nar
const uint8_t PIN_SERVO2 = 12; // Atrás Derecha 12 azu
const uint8_t PIN_SERVO3 = 13; // Atrás Izquierda 13 ver
const uint8_t PIN_POWER_SWITCH = 25; // Control del MOSFET

TouchScreen ts = TouchScreen(PIN_XP, PIN_YP, PIN_XM, PIN_YM, 500);
Servo s1, s2, s3;

float inputX, outputX, setpointX = 550; // 550
float inputY, outputY, setpointY = 550;
float tiempo;

//float KpX = 1.014, KiX = 0.0, KdX = 0.1641; 
//float KpY = 0.3075, KiY = 0.0, KdY = 0.0646;
//float KpX = 0.04744, KiX = 0.0, KdX = 0.01831; 
//float KpY = 0.2227, KiY = 0.0, KdY = 0.1204;

//float KpY = 0.019, KiY = 0.0, KdY = 0.0085; //0.025 0.01
//float KpX = 0.0136, KiX = 0.0, KdX = 0.00715; //0.02370 0.00715
//float KpX = 0.01870, KiX = 0.0, KdX = 0.00415; 

//pruebas
float KpY = 0.020, KiY = 0.001, KdY = 0.0085; //0.025 0.01
float KpX = 0.01350, KiX = 0.001, KdX = 0.0071; //0.02370 0.00715

//float KpY = 0.021, KiY = 0.0, KdY = 0.0083; //0.025 0.01
//float KpX = 0.01250, KiX = 0.0, KdX = 0.007; //0.02370 0.00715

//float KpY = 0.02, KiY = 0.0, KdY = 0.0082; //0.025 0.01
//float KpX = 0.0122, KiX = 0.0, KdX = 0.0069; //0.02370 0.00715

//float KpY = 0.019, KiY = 0.0, KdY = 0.0086; //0.025 0.01
//float KpX = 0.0130, KiX = 0.0, KdX = 0.0073; //0.02370 0.00715

//

//float KpY = 0.03463, KiY = 0.0, KdY = 0.002768;
//float KpX = 0.6921, KiX = 0.0, KdX = 0.03756;

QuickPID pidX(&inputX, &outputX, &setpointX);
QuickPID pidY(&inputY, &outputY, &setpointY);

uint32_t tiempoUltimo = 0;
const uint8_t sampleTime = 20;

//const uint8_t PIN_SWITCH = 25; // Pin físico donde conectarás el switch
//bool sistemaEncendido = false; // Variable que guarda el estado

//Filtro EMA
float alpha = 0.85; // Factor de suavizado (0.95)
float emaX = 550;  // Inicia en el centro
float emaY = 550;

//---------------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  analogReadResolution(10);

  //pinMode(PIN_SWITCH, INPUT_PULLUP);
  //Se le asignan los GPIOs a los servos
  s1.attach(PIN_SERVO1);
  s2.attach(PIN_SERVO2);
  s3.attach(PIN_SERVO3);

  //Inicio de los servos en 140 grados con espera
  s1.write(140); 
  s2.write(140);
  s3.write(135); //132
  delay(500); 

  pidX.SetTunings(KpX, KiX, KdX);
  pidY.SetTunings(KpY, KiY, KdY);
  
  pidX.SetMode(QuickPID::Control::automatic);
  pidY.SetMode(QuickPID::Control::automatic);
  
  //Antiwindup a cambiar
  pidX.SetOutputLimits(-40, 40); //-30 30
  pidY.SetOutputLimits(-40, 40);

  pinMode(PIN_POWER_SWITCH, OUTPUT);
  digitalWrite(PIN_POWER_SWITCH, HIGH); // Inicia apagado
}
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void loop() {
  uint32_t tiempoAhora = millis();
  //sistemaEncendido = !digitalRead(PIN_SWITCH);

  if (tiempoAhora - tiempoUltimo >= sampleTime) {
    tiempoUltimo = tiempoAhora;

    TSPoint p = ts.getPoint();
    
    if (p.z>=0 && p.y<1001) { //Con bola
      //Filtro EMA
      emaX = (p.x * alpha) + (emaX * (1.0 - alpha));
      emaY = (p.y * alpha) + (emaY * (1.0 - alpha));
      
      inputX = emaX;
      inputY = emaY;
      tiempo = tiempo + 0.02;
        
      Serial.print("X: ");
      Serial.print(p.x);
      Serial.print(" | Y: ");
      Serial.print(p.y);
      Serial.print(" | Presion: ");
      Serial.print(p.z); // println al final para saltar a la siguiente línea
      Serial.print(" | T (s): ");
      Serial.println(tiempo);

      //Calculo del feedback
      pidX.Compute();
      pidY.Compute();

      actualizarPlataforma(outputX, outputY);
    } else { //Sin bola
      actualizarPlataforma(0, 0); 
      tiempo = 0;
    }
  }
}
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void actualizarPlataforma(float outX, float outY) {
  const uint8_t HOME = 140; //-8
  const float FACTOR_Y = sqrt(3) / 2.0;

  // --- CAMBIO AQUÍ PARA QUE EL SERVO 1 MUEVA EL EJE X ---
  float val1 = HOME - outX;
  float val2 = HOME + (0.5 * outX) + (FACTOR_Y * outY);
  float val3 = HOME - 5 + (0.5 * outX) - (FACTOR_Y * outY);
  // -----------------------------------------------------

  s1.write(constrain(val1, 120, 160)); //120-160
  s2.write(constrain(val2, 120, 160));
  s3.write(constrain(val3, 115, 155)); //112-152
}
//---------------------------------------------------------------------------------------------------------------------
