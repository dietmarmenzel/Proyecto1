//************************************************************************************
//Universidad del Valle de Guatemala
//BE3015 - Electronica Digital 2
//Nombre: Gunther Dietmar Menzel - 19106
//PROYECTO 1
//************************************************************************************


//************************************************************************************
//Librería
//************************************************************************************
#include <Arduino.h>

//Adafruit IO key
#define IO_USERNAME "guntherdietmar" //Mi usuario
#define IO_KEY "aio_ymKI04ClsoNBp4qMGNFHRSg3SJsQ" //Mi IO KEY

//*********************************** WIFI *******************************************

//Datos de la señal WIFI
#define WIFI_SSID "mensajera1" //Nombre de mi red de WIFI
#define WIFI_PASS "since1998" //Contraseña de mi red de WIFI

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

int Decenas = 0; //Valor inicial

int Unidades = 0; //Valor inicial

int Decimales = 0; //Valor inicial

//Datos para Adafruit
int count = 0;

long LastTime; 

int sampleTime = 3000;

//************************************************************************************
//Configuracion
//************************************************************************************
void setup() {
  //Datos obtenidos del documento Publish para conectar a Adafruit
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
  //Conectando y enviando a Adafruit y enseñando en Monitor Serie
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
  digitalWrite(Transistor1, HIGH); //Transistor 1 encendido, los demás apagados
  digitalWrite(Transistor2, LOW);
  digitalWrite(Transistor3, LOW);
  Displays(Decenas); //Transistor 1 enseñando el número de decenas
  delay(5);
  
  //Configurando el estado de la salida de los Transistores
  digitalWrite(Transistor1, LOW);
  digitalWrite(Transistor2, HIGH); //Transistor 2 encendido, los demás apagados
  digitalWrite(Transistor3, LOW);
  Displays(Unidades); //Transistor 2 enseñando el número de unidades
  delay(5);
  
  //Configurando el estado de la salida de los Transistores
  digitalWrite(Transistor1, LOW);
  digitalWrite(Transistor2, LOW);
  digitalWrite(Transistor3, HIGH); //Transistor 3 encendido, los demás apagados
  Displays(Decimales); //Transistor 3 enseñando el número de decimales
  delay(5);

  MedicionDeTemperatura();
  SemaforoDeTemperatura();
  RelojDeSemaforo();
  delay(10);
}

//Configurando el despliegue del resultado en las Displays
void Displays(int Resultado){
  Decenas = Temperatura/10; //Fórmula para las decenas
  Serial.println("Decenas");
  Serial.println(Decenas); //Imprimir en el monitor serie las decenas de la temperatura
  Unidades = Temperatura-Decenas*10; //Fórmula para las unidades
  Serial.println("Unidades"); 
  Serial.println(Unidades); //Imprimir en el monitor serie las unidades de la temperatura
  Decimales = (Temperatura*10)-(Decenas*100)-(Unidades*10); //Fórmula para los decimales
  Serial.println("Decimales");
  Serial.println(Decimales); //Imprimir en el monitor serie los decimales de la temperatura
  
    //Inicializando la función switch para que compare el valor de la variablecon los valores especificados en las instrucciones case
    switch (Resultado)
    {
  //Para el resultado 0, el segmento G se apaga y las demás se encienden
  case 0:
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW);
    digitalWrite(D, LOW);
    digitalWrite(E, LOW);
    digitalWrite(F, LOW);
    digitalWrite(G, HIGH);
    break;

  //Para el resultado 1, el segmento A, D, E, F y G se apagan y las demás se encienden
  case 1:
    digitalWrite(A, HIGH);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW);
    digitalWrite(D, HIGH);
    digitalWrite(E, HIGH);
    digitalWrite(F, HIGH);
    digitalWrite(G, HIGH);
    break;
  
  //Para el resultado 2, el segmento C y F se apagan y las demás se encienden  
 case 2: 
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, HIGH);
    digitalWrite(D, LOW);
    digitalWrite(E, LOW);
    digitalWrite(F, HIGH);
    digitalWrite(G, LOW);
    break;

  //Para el resultado 3, el segmento E y F se apagan y las demás se encienden 
  case 3:  
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW);
    digitalWrite(D, LOW);
    digitalWrite(E, HIGH);
    digitalWrite(F, HIGH);
    digitalWrite(G, LOW);
    break;

  //Para el resultado 4, el segmento A, D y E se apagan y las demás se encienden 
  case 4:  
    digitalWrite(A, HIGH);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW);
    digitalWrite(D, HIGH);
    digitalWrite(E, HIGH);
    digitalWrite(F, LOW);
    digitalWrite(G, LOW);
    break;
  
  //Para el resultado 5, el segmento B y E se apagan y las demás se encienden
  case 5:  
    digitalWrite(A, LOW);
    digitalWrite(B, HIGH);
    digitalWrite(C, LOW);
    digitalWrite(D, LOW);
    digitalWrite(E, HIGH);
    digitalWrite(F, LOW);
    digitalWrite(G, LOW);
    break;
  
  //Para el resultado 6, el segmento B se apaga y las demás se encienden
  case 6:  
    digitalWrite(A, LOW);
    digitalWrite(B, HIGH);
    digitalWrite(C, LOW);
    digitalWrite(D, LOW);
    digitalWrite(E, LOW);
    digitalWrite(F, LOW);
    digitalWrite(G, LOW);
    break;
  
  //Para el resultado 7, el segmento D, E, F y G se apagan y las demás se encienden
  case 7: 
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW);
    digitalWrite(D, HIGH);
    digitalWrite(E, HIGH);
    digitalWrite(F, HIGH);
    digitalWrite(G, HIGH);
    break;
  
  //Para el resultado 8, todos los segmentos se encienden
  case 8: 
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW);
    digitalWrite(D, LOW);
    digitalWrite(E, LOW);
    digitalWrite(F, LOW);
    digitalWrite(G, LOW);
    break;
  
  //Para el resultado 9, el segmento E se apaga y las demás se encienden 
  case 9:  
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    digitalWrite(C, LOW);
    digitalWrite(D, LOW);
    digitalWrite(E, HIGH);
    digitalWrite(F, LOW);
    digitalWrite(G, LOW);
    break;
  
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
  //El servo se moverá hacia el área de color verde
  if (Temperatura < 37.0) {
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