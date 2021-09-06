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

//Adafruit IO key
#define IO_USERNAME "guntherdietmar"
#define IO_KEY "aio_ymKI04ClsoNBp4qMGNFHRSg3SJsQ"

//*********************************** WIFI *******************************************

//Datos de la señal WIFI
#define WIFI_SSID "mensajera1"
#define WIFI_PASS "since1998"

//Librería de Adafruit
#include "AdafruitIO_WiFi.h"  
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

AdafruitIO_Feed *termometro = io.feed("Proyecto Sensor de Temperatura");

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
//Función creada para el sensor de temperatura
void MedicionDeTemperatura (void);

//Función creada para las LEDs (semáforo de temperatura)
void SemaforoDeTemperatura (void);

//Función creada para configurar el Servo
void ConfiguracionDelServo (void);

//Función creada para el posicionamiento del Servo
void RelojDeSemaforo (void);

//Función creada para el resultado de las Displays
void Displays (int Resultado);

//************************************************************************************
//Variables globales
//************************************************************************************
float Temperatura = 0.0; //Definiendo float para poder tener decimales

float dutycycle = 0.0; //El ciclo del trabajo del Servo

int Resultado = 0; //Valor inicial

int Decenas = 0;

int Unidades = 0;

int Decimales = 0;

//Datos para Adafruit
int count = 0;

long LastTime; 

int sampleTime = 3000;

//************************************************************************************
//Configuracion
//************************************************************************************
void setup() {
  //Datos obtenidos del documento Publish para Adafruit
  while(! Serial);

  Serial.print("Connecting to Adafruit IO");
  io.connect();

  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println();
  Serial.println(io.statusText());
  LastTime = millis();

  //Configuracion a cada Pin de entrada
  pinMode(Boton, INPUT);
  pinMode(Sensor, INPUT);
  //Configuracion a cada Pin de salida
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

  ConfiguracionDelServo (); //Definiendo parámetros del servo
  Serial.begin(115200);

}


//************************************************************************************
//Loop principal
//************************************************************************************
  void loop() {

  if (millis()- LastTime >= sampleTime){
    io.run();
 
    Serial.print("sending -> ");
    Serial.println(count);
    termometro->save(Temperatura);

    count++;
    
    LastTime = millis();
  }

  Displays(Resultado);
  
  //Configurando el estado de la salida de los Transistores
  digitalWrite(Transistor1, HIGH);
  digitalWrite(Transistor2, LOW);
  digitalWrite(Transistor3, LOW);
  Displays(Decenas);
  delay(10);
  
  //Configurando el estado de la salida de los Transistores
  digitalWrite(Transistor1, LOW);
  digitalWrite(Transistor2, HIGH);
  digitalWrite(Transistor3, LOW);
  Displays(Unidades);
  delay(10);
  
  //Configurando el estado de la salida de los Transistores
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

//Configurando el despliegue del resultado en las Displays
void Displays(int Resultado){
  Decenas = Temperatura/10;
  Unidades = Temperatura-Decenas*10;
  Decimales = ((Temperatura*1000)-(Decenas*1000)-(Unidades*100))/1000;
  
  //Para el resultado 0, el segmento G se apaga y las demás se encienden
  if (digitalRead(Resultado)==0){

    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW);
    digitalWrite(D, LOW);
    digitalWrite(E, LOW);
    digitalWrite(F, LOW);
    digitalWrite(G, HIGH);
    
  }
  //Para el resultado 1, el segmento A, D, E, F y G se apagan y las demás se encienden
  else if(digitalRead(Resultado)==1){
    digitalWrite(A, HIGH);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW);
    digitalWrite(D, HIGH);
    digitalWrite(E, HIGH);
    digitalWrite(F, HIGH);
    digitalWrite(G, HIGH);
  }
  //Para el resultado 2, el segmento C y F se apagan y las demás se encienden  
  else if(digitalRead(Resultado)==2){  
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, HIGH);
    digitalWrite(D, LOW);
    digitalWrite(E, LOW);
    digitalWrite(F, HIGH);
    digitalWrite(G, LOW);
  }
  //Para el resultado 3, el segmento E y F se apagan y las demás se encienden 
  else if(digitalRead(Resultado)==3){  
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW);
    digitalWrite(D, LOW);
    digitalWrite(E, HIGH);
    digitalWrite(F, HIGH);
    digitalWrite(G, LOW);
  }
  //Para el resultado 4, el segmento A, D y E se apagan y las demás se encienden 
  else if(digitalRead(Resultado)==4){  
    digitalWrite(A, HIGH);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW);
    digitalWrite(D, HIGH);
    digitalWrite(E, HIGH);
    digitalWrite(F, LOW);
    digitalWrite(G, LOW);
  }
  //Para el resultado 5, el segmento B y E se apagan y las demás se encienden
  else if(digitalRead(Resultado)==5){  
    digitalWrite(A, LOW);
    digitalWrite(B, HIGH);
    digitalWrite(C, LOW);
    digitalWrite(D, LOW);
    digitalWrite(E, HIGH);
    digitalWrite(F, LOW);
    digitalWrite(G, LOW);
  }
  //Para el resultado 6, el segmento B se apaga y las demás se encienden
  else if(digitalRead(Resultado)==6){  
    digitalWrite(A, LOW);
    digitalWrite(B, HIGH);
    digitalWrite(C, LOW);
    digitalWrite(D, LOW);
    digitalWrite(E, LOW);
    digitalWrite(F, LOW);
    digitalWrite(G, LOW);
  }
  //Para el resultado 7, el segmento D, E, F y G se apagan y las demás se encienden
  else if(digitalRead(Resultado)==7){  
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW);
    digitalWrite(D, HIGH);
    digitalWrite(E, HIGH);
    digitalWrite(F, HIGH);
    digitalWrite(G, HIGH);
  }
  //Para el resultado 8, todos los segmentos se encienden
  else if(digitalRead(Resultado)==8){  
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW);
    digitalWrite(D, LOW);
    digitalWrite(E, LOW);
    digitalWrite(F, LOW);
    digitalWrite(G, LOW);
  }
  //Para el resultado 9, el segmento E se apaga y las demás se encienden 
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


//Llamando a la función del sensor de temperatura
void MedicionDeTemperatura (void) {
  if (digitalRead(Boton) == LOW) {
    Temperatura = analogRead(Sensor); //Leer valor analogo del sensor
    Temperatura = Temperatura/10;
    Serial.println(Temperatura); //Mostrar temperatura en el Monitor Serie
  }
}

//Definiendo bajo que parámetros estarán encendiadas las LEDs
void SemaforoDeTemperatura (void) {
  if (Temperatura < 37.0) {
    digitalWrite(LEDverde, HIGH); //LED verde se enciende, las demás se apagan
    digitalWrite(LEDamarillo, LOW);
    digitalWrite(LEDrojo, LOW);
  }

  if (Temperatura > 37.0 & Temperatura < 37.5) {
    digitalWrite(LEDverde, LOW);
    digitalWrite(LEDamarillo, HIGH); //LED amarillo se enciende, las demás se apagan
    digitalWrite(LEDrojo, LOW);
  }

  if (Temperatura > 37.5) {
    digitalWrite(LEDverde, LOW);
    digitalWrite(LEDamarillo, LOW);
    digitalWrite(LEDrojo, HIGH); //LED rojo se enciende, las demás se apagan
  }

}

//Llamando a la función del movimiento del servo y configurándolo con su canal, frecuencia y resolución
void ConfiguracionDelServo (void) {
  ledcSetup(CanalServoMotor, Frecuencia, Resolucion);
  ledcAttachPin(ServoMotor, CanalServoMotor);

}
//Definiendo bajo que parámetros y hacia donde se moverá el Servo
void RelojDeSemaforo (void) {
  if (Temperatura < 37.0) {
    //El servo se moverá hacia el área de color verde
    dutycycle = 5; //Ciclo del servo
    delay(20);
    ledcWrite(CanalServoMotor, dutycycle); //Trabajo del PWM


  }
  //El servo se moverá hacia el área de color amarillo
  if (Temperatura > 37.0 & Temperatura < 37.5) {
    dutycycle = 19; //Ciclo del servo
    delay(20);
    ledcWrite(CanalServoMotor, dutycycle); //Trabajo del PWM

  }
  //El servo se moverá hacia el área de color rojo
  if (Temperatura > 37.5) {
    dutycycle = 25; //Ciclo del servo
    delay(20);
    ledcWrite(CanalServoMotor, dutycycle); //Trabajo del PWM

  }
  }

  //FIN DEL CÓDIGO