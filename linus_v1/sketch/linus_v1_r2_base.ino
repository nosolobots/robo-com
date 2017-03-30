// -----------------------------------------------------------------------------
//               				L I N U S - V.1 - R2 - BASE
// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
//
//  Copyright (c) 2016, Fran Montoiro <zeroth@nosolobots.org>
//
//  License: BSD-3-Clause
//
// -----------------------------------------------------------------------------
//  Changelog:
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Librerías
// -----------------------------------------------------------------------------

#include <QTRSensors.h>

// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Definiciones
// -----------------------------------------------------------------------------

// TB6612FNG PIN's
// ------------------
#define PIN_BIN1  		7
#define PIN_BIN2  		8
#define PIN_PWMB  		9
#define PIN_AIN1  		5
#define PIN_AIN2  		4
#define PIN_PWMA  		3
#define PIN_STBY  		6

// LED's
#define PIN_L1  		0

// Switches
#define PIN_SW1  		10
#define PIN_SW2  		11

// Velocidad base del motor
#define MOT_BASE_SPEED	200

// Retardo
#define DELAY_START		2000  // Retardo en ms antes de iniciar la carrera
#define DELAY_STOP		2000  // Retardo tras la detención de los motores

// QTR-8RC
#define NUM_SENSORS   	8     // Número de sensores uasados
#define TIMEOUT       	2500  // Espera 2500 ms para que los sensores pasen a baja
#define EMITTER_PIN   	13    // Pin digital del Emitter del QTR8-RC
#define CENTER_POS		  3500  // Lectura de los sensores cuando está centrado sobre la línea#define CALIB_TIME      3     // Tiempo en segundos de calibrado
#define CALIB_TIME      3     // Tiempo en segundos de calibrado

// PID function
#define KP       		0.06  	// Proportional
#define KD       		0.5     // Derivative
#define KI				  0.0	  	// Integral	

// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Global

// QTR-8RC object: analog PIN's A0(14)-A5(19)
QTRSensorsRC qtrrc( (unsigned char[]) {2, 19, 18, 17, 16, 15, 14, 12},
                    NUM_SENSORS,
                    TIMEOUT,
                    EMITTER_PIN);

// QTR-8RC array de los valores de los sensores
unsigned int sensor_values[NUM_SENSORS];

// PID variables
int integral = 0;
int last_error = 0;

// Flags
int calibrated = 0;		// Indica si los sensores están calibrados
int running = 0;		  // Estado de "en carrera"

// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Funciones
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
void doCalibration()
// -----------------------------------------------------------------------------
// Calibra los sensores del QTR8RC durante 5 seg aprox.
// Activa el LED durante el proceso
{
	delay(1000);
	
	digitalWrite(PIN_L1, HIGH);     // Activación del LED

  // Realiza la calibración durante el núm de seg especificado en CALIB_TIME
  // Cada llamada a calibrate() lee los sensores 10 veces con un freq de 2500us (25ms aprox)
	for (int i = 0, ct = CALIB_TIME*40; i < ct; i++) { 
	  qtrrc.calibrate();                                
  }	                                                  

	digitalWrite(PIN_L1, LOW);      // Apagar LED
}

// -----------------------------------------------------------------------------
void moveForward()
// -----------------------------------------------------------------------------
// Obtine la posición actual del robot respecto a la línea y calcula el incremento
// de la velocidad de los motores para corregir su trayectoria
{
	int l_motor_speed, r_motor_speed, position, error;
	int motor_speed;

	// Obtenemos la posición actual
	position = qtrrc.readLine(sensor_values);

	// Calculamos los parámetros de la función PID para actualizar la posición
	error = position - CENTER_POS;
	integral += last_error;
	//integral = constrain(integral, -1000, 1000);
	motor_speed = KP * error + KD * (error - last_error) + KI * integral;
	last_error = error;

  // Actualizamos la velocidad de los motores
  l_motor_speed = MOT_BASE_SPEED + motor_speed;
	r_motor_speed = MOT_BASE_SPEED - motor_speed;

	l_motor_speed = constrain(l_motor_speed, 0, 254);
	r_motor_speed = constrain(r_motor_speed, 0, 254);

	// Aplicamos las nuevas velocidades a los motores
	digitalWrite(PIN_AIN1, HIGH);
	digitalWrite(PIN_AIN2, LOW);
	digitalWrite(PIN_BIN1, LOW);
	digitalWrite(PIN_BIN2, HIGH);
	analogWrite(PIN_PWMA, l_motor_speed);
	analogWrite(PIN_PWMB, r_motor_speed);
}

// -----------------------------------------------------------------------------
void stopMotors()
// -----------------------------------------------------------------------------
{
	analogWrite(PIN_PWMA, 0);
	analogWrite(PIN_AIN1, 0);
	analogWrite(PIN_AIN2, 0);
	analogWrite(PIN_PWMB, 0);
	analogWrite(PIN_BIN1, 0);
	analogWrite(PIN_BIN2, 0);
	analogWrite(PIN_STBY, 0);
}

// -----------------------------------------------------------------------------
void setup()
// -----------------------------------------------------------------------------
// Inicialización 
{
	pinMode(PIN_SW1, INPUT_PULLUP);
	pinMode(PIN_SW2, INPUT_PULLUP);
	pinMode(PIN_L1, OUTPUT);
	
	digitalWrite(PIN_L1, LOW);
}

// -----------------------------------------------------------------------------
void loop()
// -----------------------------------------------------------------------------
// Bucle de inicialización principal
{
  while(!calibrated){
	  // Check Button 1
	  if(digitalRead(PIN_SW1) == LOW) {
		  doCalibration();	// Calibra los sensores
		  calibrated = 1;
	  }
	}

	// Check Button 2
	if(digitalRead(PIN_SW2) == LOW)	{
		if(running) {
			running = 0;
			stopMotors(); 		// Detiene el vehículo
			delay(DELAY_STOP);
		}
		else {
			running = 1;		// Start running
			digitalWrite(PIN_STBY, HIGH);
			delay(DELAY_START);
		}
	}
	
	if(running) {
		moveForward();
	}
}
