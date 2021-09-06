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

void Displays (int Resultado);

//************************************************************************************
//Variables globales
//************************************************************************************
float Temperatura = 0.0; //Definiendo float para poder

float dutycycle = 0.0; //

int Resultado = 0; //valor inicial

int Decenas = 0;

int Unidades = 0;

int Decimales = 0;

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

  MedicionDeTemperatura();
  SemaforoDeTemperatura();
  RelojDeSemaforo();
  delay(1);
}

void Displays(int Resultado){
  Decenas = Temperatura/10;
  Unidades = Temperatura-Decenas*10;
  Decimales = ((Temperatura*1000)-(Decenas*1000)-(Unidades*100))/1000;
  if (digitalRead(Resultado)==0){

        digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW);
    digitalWrite(D, LOW);
    digitalWrite(E, LOW);
    digitalWrite(F, LOW);
    digitalWrite(G, HIGH);
    
  }
  else if(digitalRead(Resultado)==1){
    digitalWrite(A, HIGH);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW);
    digitalWrite(D, HIGH);
    digitalWrite(E, HIGH);
    digitalWrite(F, HIGH);
    digitalWrite(G, HIGH);
  }
    
  else if(digitalRead(Resultado)==2){  
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, HIGH);
    digitalWrite(D, LOW);
    digitalWrite(E, LOW);
    digitalWrite(F, HIGH);
    digitalWrite(G, LOW);
  }
  else if(digitalRead(Resultado)==3){  
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW);
    digitalWrite(D, LOW);
    digitalWrite(E, HIGH);
    digitalWrite(F, HIGH);
    digitalWrite(G, LOW);
  }
  else if(digitalRead(Resultado)==4){  
    digitalWrite(A, HIGH);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW);
    digitalWrite(D, HIGH);
    digitalWrite(E, HIGH);
    digitalWrite(F, LOW);
    digitalWrite(G, LOW);
  }
  else if(digitalRead(Resultado)==5){  
    digitalWrite(A, LOW);
    digitalWrite(B, HIGH);
    digitalWrite(C, LOW);
    digitalWrite(D, LOW);
    digitalWrite(E, HIGH);
    digitalWrite(F, LOW);
    digitalWrite(G, LOW);
  }
  else if(digitalRead(Resultado)==6){  
    digitalWrite(A, LOW);
    digitalWrite(B, HIGH);
    digitalWrite(C, LOW);
    digitalWrite(D, LOW);
    digitalWrite(E, LOW);
    digitalWrite(F, LOW);
    digitalWrite(G, LOW);
  }
  else if(digitalRead(Resultado)==7){  
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW);
    digitalWrite(D, HIGH);
    digitalWrite(E, HIGH);
    digitalWrite(F, HIGH);
    digitalWrite(G, HIGH);
  }
  else if(digitalRead(Resultado)==8){  
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW);
    digitalWrite(D, LOW);
    digitalWrite(E, LOW);
    digitalWrite(F, LOW);
    digitalWrite(G, LOW);
  }
  else if(digitalRead(Resultado)==9){  
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW);
    digitalWrite(D, LOW);
    digitalWrite(E, HIGH);
    digitalWrite(F, LOW);
    digitalWrite(G, LOW);
  }
  }



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

void ConfiguracionDelServo (void) {
  ledcSetup(CanalServoMotor, Frecuencia, Resolucion);
  ledcAttachPin(ServoMotor, CanalServoMotor);

}

void RelojDeSemaforo (void) {
  if (Temperatura < 37.0) {
    dutycycle = 5; //ciclo del servo
    delay(20);
    ledcWrite(CanalServoMotor, dutycycle); //Trabajo del PWM


  }

  if (Temperatura > 37.0 & Temperatura < 37.5) {
    dutycycle = 19; //ciclo del servo
    delay(20);
    ledcWrite(CanalServoMotor, dutycycle); //Trabajo del PWM

  }

  if (Temperatura > 37.5) {
    dutycycle = 25; //ciclo del servo
    delay(20);
    ledcWrite(CanalServoMotor, dutycycle); //Trabajo del PWM

  }
  }