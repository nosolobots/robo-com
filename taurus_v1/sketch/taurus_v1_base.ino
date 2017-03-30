//  taurus_v1_base.ino
// -----------------------------------------------------------------------------
//               				T A U R U S - V.1 - B A S E
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
//
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Librerías
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Definiciones
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// PIN Numbers

// TB6612FNG 
// ------------------
#define PIN_BIN1  		5
#define PIN_BIN2  		4
#define PIN_PWMB  		3
#define PIN_AIN1  		7
#define PIN_AIN2  		8
#define PIN_PWMA  		9
#define PIN_STBY  		6

// CNY70
// ------------------
#define PIN_CNY70_FL	10      // front-left
#define PIN_CNY70_FR	11      // front-right
#define PIN_CNY70_RR	12      // rear-right
#define PIN_CNY70_RL	13      // rear-left

// GP2Y0A21YK0F
// ------------------
#define PIN_GP2Y0_FL	16      // front-left (A2)
#define PIN_GP2Y0_L 	17      // left (A3)
#define PIN_GP2Y0_R   18      // right (A4)
#define PIN_GP2Y0_FR	19      // front-right (A5)

// LED's
// ------------------
#define PIN_L1  		1
#define PIN_L2  		0

// Switches
// ------------------
#define PIN_SW1  		14
#define PIN_SW2  		15

// -----------------------------------------------------------------------------
// Constantes

// Umbral de detección para los GP2Y0
#define FOE_FRN_DETECT        160          
#define FOE_LAT_DETECT        160          

// Velocidades base del motor
#define BASE_SPEED	    180
#define RAM_SPEED	      240
#define ROT_SPEED       220
#define KP              0.35

// Flags de detección de CNY70
#define CNY70_FL	    1      // front-left
#define CNY70_FR	    2      // front-right
#define CNY70_RL	    4      // rear-left
#define CNY70_RR	    8      // rear-right

// Rotaciones
#define ROT_90L         1
#define ROT_90R         2
#define ROT_135L        3
#define ROT_135R        4
#define ROT_180L        5
#define ROT_180R        6

// Modos de operación
#define OP_NUM          4       // Num modos
#define OP_SEARCH       1
#define OP_TEST_MOT     2
#define OP_TEST_GPY     3
#define OP_TEST_CNY70   4

// Delay (ms)
#define START_DELAY     5000
#define SEARCH_DELAY    1000

// -----------------------------------------------------------------------------
// Globales

// Flag de activación
int running = 0;

// Flag de "sobre línea blanca"
int on_border = 0;

// Flag de modo de op
int selected_mode = 0;

// rotation_matrix { r_mot(in1, in2), l_mot(in1, in2), delay }
int rot_matrix[][5] = {
    { 0, 0, 0, 0, 0 },      // dummy
    { 1, 0, 0, 1, 150 },    // left turn 90
    { 0, 1, 1, 0, 150 },    // right turn 90
    { 1, 0, 0, 1, 210 },    // left turn 135
    { 0, 1, 1, 0, 210 },    // right turn 135    
    { 1, 0, 0, 1, 340 },    // left turn 180
    { 0, 1, 1, 0, 340 },    // right turn 180
};

// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Funciones
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
int checkDojoBorder()
// -----------------------------------------------------------------------------
// Chequea los sensores de suelo para detectar si están sobre el borde (HIGH) 
// Devuelve un entero que indica sensores activos
{
  int ret=0;
  
  if(digitalRead(PIN_CNY70_FR))
    ret += CNY70_FR;
  
  if(digitalRead(PIN_CNY70_FL))
    ret += CNY70_FL;
  
  if(digitalRead(PIN_CNY70_RR))
    ret += CNY70_RR;
  
  if(digitalRead(PIN_CNY70_RL))
    ret += CNY70_RL;
  
  return ret;
}

// -----------------------------------------------------------------------------
int checkGP2Y0(int sensor)
// -----------------------------------------------------------------------------
// Obtiene la distancia medida por el sensor
// Devuelve el promedio de varias lecturas consecutivas
{
  int i, val=0;
    
  for(i=0; i<8; i++){
    val += analogRead(sensor);
  }
    
  val /= 8;
    
  return val;
}

// -----------------------------------------------------------------------------
void testCNY70()
// -----------------------------------------------------------------------------
// Testeo de los sensores de suelo. 
// Si alguno de los sensores está sobre el borde se activa el led L1
// El testeo finaliza al pulsar el botón SW_2
{
  int val;
  
  while(running){
    val = checkDojoBorder();
    
    if(val)
      digitalWrite(PIN_L1, HIGH);
    else
      digitalWrite(PIN_L1, LOW);

    if(digitalRead(PIN_SW2)==LOW){
      delay(250);
  
      reset();
    }
  }
}

// -----------------------------------------------------------------------------
void testGP2Y0()
// -----------------------------------------------------------------------------
// Testeo de los sensores de distancia
// Si hay detencción en los sensores de la derecha se ilumina el led L1 
// Si hay detencción en los sensores de la izquierda se ilumina el led L2
// El testeo finaliza al pulsar el botón SW_2
{
  int r, fr, fl, l;
  int led;
  
  while(running){
    r = checkGP2Y0(PIN_GP2Y0_R);
    fr = checkGP2Y0(PIN_GP2Y0_FR);
    fl = checkGP2Y0(PIN_GP2Y0_FL);
    l = checkGP2Y0(PIN_GP2Y0_L);
    
    led=(r>FOE_LAT_DETECT || fr>FOE_FRN_DETECT)?HIGH:LOW;
    digitalWrite(PIN_L1, led);
    led=(l>FOE_LAT_DETECT || fl>FOE_FRN_DETECT)?HIGH:LOW;
    digitalWrite(PIN_L2, led);
    
    if(digitalRead(PIN_SW2)==LOW){
      delay(250);
    
      reset();
    }
  }   
}

// -----------------------------------------------------------------------------
void testMotors()
// -----------------------------------------------------------------------------
// Testea los motores realizando un avance y un retroceso
{
	// Aplicamos las nuevas velocidades a los motores
	digitalWrite(PIN_AIN1, HIGH);
	digitalWrite(PIN_AIN2, LOW);
	digitalWrite(PIN_BIN1, HIGH);
	digitalWrite(PIN_BIN2, LOW);
	digitalWrite(PIN_STBY, HIGH);
	analogWrite(PIN_PWMA, BASE_SPEED);
	analogWrite(PIN_PWMB, BASE_SPEED);    
	
	delay(1500);
	
	stopMotors();
	delay(1000);
	
	digitalWrite(PIN_AIN1, LOW);
	digitalWrite(PIN_AIN2, HIGH);
	digitalWrite(PIN_BIN1, LOW);
	digitalWrite(PIN_BIN2, HIGH);
	digitalWrite(PIN_STBY, HIGH);
	analogWrite(PIN_PWMA, BASE_SPEED);
	analogWrite(PIN_PWMB, BASE_SPEED);     
	
	delay(1500);
	
	stopMotors();
	delay(1000);
	
	reset();
}

// -----------------------------------------------------------------------------
void search_n_destroy()
// -----------------------------------------------------------------------------
// Trata de localizar al oponente y se abalanza sobre él
{
  int r, fr, fl, l;
  int led, error = 0, foe = 0, border;
  int rmot_speed, lmot_speed;
  unsigned long cur_time;
  int foe_distance;
  
  foe_distance = 0;
  
  while(running){
    rmot_speed = BASE_SPEED;
    lmot_speed = BASE_SPEED;
    
    // Check Dojo border
    border = checkDojoBorder();
    if(border) {
      stopMotors();
      advance(-RAM_SPEED, -RAM_SPEED);
      delay(250);

      // Rotamos para evitar el borde
      switch(border){
        case CNY70_FR:
          rotate(ROT_135L);
          break;
        case CNY70_FL:
          rotate(ROT_135R);
          break;
        case CNY70_FL + CNY70_FR:
          rotate(ROT_180L);
          break;
      }
    }

    // Leemos los sensores distancia
    r = checkGP2Y0(PIN_GP2Y0_R);
    fl = checkGP2Y0(PIN_GP2Y0_FL);
    fr = checkGP2Y0(PIN_GP2Y0_FR);
    l = checkGP2Y0(PIN_GP2Y0_L);

    // Check oponente al frente
    if(fl > FOE_FRN_DETECT || fr > FOE_FRN_DETECT){
      // Descubrimos al enemigo!!!
      foe = 1;
      digitalWrite(PIN_L1, HIGH);
      
      foe_distance = (fl>=fr)?fl:fr;
      
      // Ajustamos las velocidades
      error = fr - fl;
      error *= KP;
      rmot_speed = constrain(RAM_SPEED - error, 0, 255);
      lmot_speed = constrain(RAM_SPEED + error, 0, 255);
    }          
    else{
      // No oponente al frente
      foe = 0;
      foe_distance = 0;
          
      digitalWrite(PIN_L1, LOW);            
      
      // Chequeamos si esta por un lateral
      if(r>FOE_LAT_DETECT){
        digitalWrite(PIN_L2, HIGH);
        rotate(ROT_90R);
        digitalWrite(PIN_L2, LOW);
      }
      else if(l>FOE_LAT_DETECT){
        digitalWrite(PIN_L2, HIGH);
        rotate(ROT_90L);
        digitalWrite(PIN_L2, LOW);
      }
      
    }

    advance(lmot_speed, rmot_speed);

    if(digitalRead(PIN_SW2)==LOW){
      delay(250);

      reset();
    }        
  }
}

// -----------------------------------------------------------------------------
void advance(int lspeed, int rspeed)
// -----------------------------------------------------------------------------
// Aplica las velocidades a los motores
{
  int inA1, inA2, inB1, inB2;
  
  if(rspeed>0){ 
    inA1 = HIGH; 
    inA2 = LOW;
  }
  else {
    inA1 = LOW;
    inA2 = HIGH;
  }
  
  if(lspeed>0){
    inB1 = HIGH;
    inB2 = LOW;
  }
  else {
    inB1 = LOW;
    inB2 = HIGH;
  }    
    
	// Aplicamos las nuevas velocidades a los motores
	digitalWrite(PIN_AIN1, inA1);
	digitalWrite(PIN_AIN2, inA2);
	digitalWrite(PIN_BIN1, inB1);
	digitalWrite(PIN_BIN2, inB2);
	digitalWrite(PIN_STBY, HIGH);
	analogWrite(PIN_PWMA, rspeed);
	analogWrite(PIN_PWMB, lspeed);      
}

// -----------------------------------------------------------------------------
void reset()
// -----------------------------------------------------------------------------
// Reinicia estados del robot
{
  stopMotors();
  
  running = 0;
  selected_mode = 0;
  
  digitalWrite(PIN_L1, LOW);
  digitalWrite(PIN_L2, LOW);
}

// -----------------------------------------------------------------------------
void stopMotors()
// -----------------------------------------------------------------------------
// Detiene los motores
{
	analogWrite(PIN_PWMA, 0);
	digitalWrite(PIN_AIN1, 0);
	digitalWrite(PIN_AIN2, 0);
	analogWrite(PIN_PWMB, 0);
	digitalWrite(PIN_BIN1, 0);
	digitalWrite(PIN_BIN2, 0);
	digitalWrite(PIN_STBY, 0);
}

// -----------------------------------------------------------------------------
void rotate(int rot)
// -----------------------------------------------------------------------------
// Aplica la rotación correspondiente sobre los motores
{
  int r1, r2, l1, l2, tm;

  r1 = rot_matrix[rot][0];
  r2 = rot_matrix[rot][1];
  l1 = rot_matrix[rot][2];
  l2 = rot_matrix[rot][3];
  tm = rot_matrix[rot][4];

	digitalWrite(PIN_AIN1, r1);
	digitalWrite(PIN_AIN2, r2);
	digitalWrite(PIN_BIN1, l1);
	digitalWrite(PIN_BIN2, l2);
	digitalWrite(PIN_STBY, HIGH);
	analogWrite(PIN_PWMA, ROT_SPEED);
	analogWrite(PIN_PWMB, ROT_SPEED);
	
  delay(tm);  	
}

// -----------------------------------------------------------------------------
void modeSelection()
// -----------------------------------------------------------------------------
// Utilizamos el botón SW1 para ir desplazándonos por los distintos modos
// Cada vez que seleccionamos un modo se indica visualmente activando el led L1
// Utilizamos el botón SW2 para confirmar el modo seleccionado
{
  int mode = 0, j, t = 125;
  
  while(!selected_mode) {
    if(digitalRead(PIN_SW1)==LOW){
      delay(250);
      
      mode++;
      if(mode>OP_NUM) { mode = 1; }
      
      for(j=0; j<mode; j++){
        digitalWrite(PIN_L1, HIGH);
        delay(t);
        digitalWrite(PIN_L1, LOW);
        delay(t);
      }
    }
      
    if(digitalRead(PIN_SW2)==LOW && mode){
      // Confirmación del modo seleccionado
      delay(250);
      
      for(int i=0; i<mode; i++) {
        digitalWrite(PIN_L1, HIGH);
        delay(t);
        digitalWrite(PIN_L1, LOW);
        delay(t);
      }
      
      digitalWrite(PIN_L2, HIGH);
      
      selected_mode = mode;
    }
  }
}

// -----------------------------------------------------------------------------
void setup()
// -----------------------------------------------------------------------------
// Inicialización 
{
  // Switches
	pinMode(PIN_SW1, INPUT_PULLUP);
	pinMode(PIN_SW2, INPUT_PULLUP);
	
	// CNY70 sensors
	pinMode(PIN_CNY70_RR, INPUT);
	pinMode(PIN_CNY70_FR, INPUT);
	pinMode(PIN_CNY70_FL, INPUT);
	pinMode(PIN_CNY70_RL, INPUT);

	// Led's
	pinMode(PIN_L1, OUTPUT);
	pinMode(PIN_L2, OUTPUT);	
	digitalWrite(PIN_L1, LOW);
	digitalWrite(PIN_L2, LOW);
}

// -----------------------------------------------------------------------------
void loop()
// -----------------------------------------------------------------------------
// Bucle de ejecución principal
{
  int rot;
  
  // Selección del modo de operación
  if(!selected_mode) {
    rot = 0;
    modeSelection();
  }
  
  // Pulsar el botón SW2 para iniciar ejecución del modo seleccionado
  while(!running && selected_mode) {
    if(digitalRead(PIN_SW2)==LOW){
      delay(250);
      
      int ciclos = (START_DELAY-250)/250;
      for(int i=0, led=0; i<ciclos; i++){           
        digitalWrite(PIN_L2, led);
        delay(250);
        led = (led)?0:1;
      }
      
      digitalWrite(PIN_L2, LOW);
      running = 1;
    }
  }           

  // Ejecución del modo seleccionado    
  if(running) {
    switch(selected_mode){
      case 1:    
        search_n_destroy();
        break;
      case 2:
        testMotors();
        break;
      case 3:
        testGP2Y0();
        break;
      case 4:
        testCNY70();
        break;
    }
  }
}