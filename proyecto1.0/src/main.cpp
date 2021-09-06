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