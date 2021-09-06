//************************************************************************************
//Universidad del Valle de Guatemala
//BE3015 - Electronica Digital 2
//Nombre: Gunther Dietmar Menzel
//PROYECTO 1
//************************************************************************************


//************************************************************************************
//Librería
//************************************************************************************
#include <Arduino.h>

//************************************************************************************
//Definición de pines
//************************************************************************************
//Entradas
#define Boton 34
#define Sensor 33
//Salidas
#define ServoMotor 32
#define LEDverde 25
#define LEDamarillo 26
#define LEDrojo 27
#define A 21
#define B 19
#define C 18
#define D 5
#define E 17
#define F 16
#define G 4
#define Transistor1 14
#define Transistor2 12
#define Transistor3 13

//PWM del Servo
#define CanalServoMotor 1
#define Frecuencia 50
#define Resolucion 8

//PWM de LEDverde
#define CanalLEDverde 2

//PWM de LEDamarillo
#define CanalLEDamarillo 3

//PWM de LEDrojo
#define CanalLEDrojo 4

//************************************************************************************
//Prototipos de funciones
//************************************************************************************
void MedicionDeTemperatura (void);

void SemaforoDeTemperatura (void);

void ConfiguracionDelServo (void);

void RelojDeSemaforo (void);

//************************************************************************************
//Variables globales
//************************************************************************************
float Temperatura = 0.0; //Definiendo float para poder

float dutycycle = 0.0; //

int Resultado = 0; //valor inicial


//************************************************************************************
//Configuracion
//************************************************************************************
void setup() {

  pinMode(Boton, INPUT);
  pinMode(Sensor, INPUT);

  pinMode(ServoMotor, OUTPUT);
  pinMode(LEDverde, OUTPUT);
  pinMode(LEDamarillo, OUTPUT);
  pinMode(LEDrojo, OUTPUT);
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(D, OUTPUT);
  pinMode(E, OUTPUT);
  pinMode(F, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(Transistor1, OUTPUT);
  pinMode(Transistor2, OUTPUT);
  pinMode(Transistor3, OUTPUT);

  ConfiguracionDelServo (); //Definiendo parametros del servo
  Serial.begin(115200);

}


//************************************************************************************
//Loop principal
//************************************************************************************

  digitalWrite(Transistor1, HIGH);
  digitalWrite(Transistor2, LOW);
  digitalWrite(Transistor3, LOW);
  Displays(Decenas);
  delay(10);

  digitalWrite(Transistor1, LOW);
  digitalWrite(Transistor2, HIGH);
  digitalWrite(Transistor3, LOW);
  Displays(Unidades);
  delay(10);

  digitalWrite(Transistor1, LOW);
  digitalWrite(Transistor2, LOW);
  digitalWrite(Transistor3, HIGH);
  Displays(Decimales);
  delay(10);

void MedicionDeTemperatura (void) {
  if (digitalRead(Boton) == LOW) {
    Temperatura = analogRead(Sensor); //Leer valor analogo del sensor
    Temperatura = Temperatura/10;
    Serial.println(Temperatura); 
  }
}


void SemaforoDeTemperatura (void) {
  if (Temperatura < 37.0) {
    digitalWrite(LEDverde, HIGH);
    digitalWrite(LEDamarillo, LOW);
    digitalWrite(LEDrojo, LOW);
  }

  if (Temperatura > 37.0 & Temperatura < 37.5) {
    digitalWrite(LEDverde, LOW);
    digitalWrite(LEDamarillo, HIGH);
    digitalWrite(LEDrojo, LOW);
  }

  if (Temperatura > 37.5) {
    digitalWrite(LEDverde, LOW);
    digitalWrite(LEDamarillo, LOW);
    digitalWrite(LEDrojo, HIGH);
  }

}