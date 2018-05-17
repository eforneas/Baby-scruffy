/*
 * 
 * Autor: Eduardo J. Fórneas Lence - Innovanube de Computación SL
 * 
 * comprobar que un par de BT-05 emparejados envían y reciben datos
 * 
 * el proceso se comprueba en tres pasos:
 * 
 * 1) a través del terminal serie enviamos datos que son leídos por BT_master
 * 2) BT_master envia los datos recibidos a BT_slave
 * 3) BT_slave envía los datos recibidos al monitor serial cerrando el circuito de comunicación
 *
 *  detalles del pineado de la placa (Arduino UNO)
 *  
 *  pin 10 -> al pin BT master (RX)
 *  pin 11 -> al pin BT master (TX)
 *  pin  2 -> al pin BT slave  (TX)
 *  pin  3 -> al pin BT slave  (RX)
 *  
 *  ambos deben tener suministro de corriente y deben estar en modo no programable, una vez terminada la tarea de
 *  emparejamiento, los leds de ambos módulos deben parpadear dos veces cada 2 segundos (aprox.) el módulo master
 *  primero y el slave después nada más aplicar corriente al circuito
 *  
 *  prueba de concepto, se lee un IMU y se envía a través de un bluetooth maestro a uno esclavo,
 *  la lectura aparecerá en le puerto serie
 *  
*/

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

/*
 * 
 *  if (0 <= x && x <= 3) // between
 *  if (y < 0 || 10 < y) // outside
 */
#define between(x, a, b)  (((a) <= (x)) && ((x) <= (b)))
#define outside(x, a, b)  (((x) < (a)) || ((b) < (x)))
#define DEBUGER  true
#define ARRAY_BT_COMMANDS 25

String bt_commands[ ARRAY_BT_COMMANDS ] = { "S", "f", "F", "b", "B", "r", "R", "l", "L", "q", "Q", "z", "Z", "e", "E", "c", "C", "u", "U", "d", "D", "1", "2", "3", "4" };

 // variación mínima para un cambio de valor de lectura
const float min_variation = 0.50 ;
 
const int mpuAddress = 0x68;  // Puede ser 0x68 o 0x69
MPU6050 mpu(mpuAddress);

// la constante limita el ángulo de cambio de comando fF, bB, lL, rR, etc.
const int chg_limit = 35 ;
// ángulo a partir del cual no genera comando
const int u_limit = 55 ;
// ángulo (máximo y mínimo) que se considera "plano" en parada (stop) --> no acción
const int death_angle = 10 ;
 
int ax, ay, az;
int gx, gy, gz;
 
long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

int value_code = 0 ;
char char_value ;

void updateFiltered()
{
   dt = (millis() - tiempo_prev) / 1000.0;
   tiempo_prev = millis();
 
   //Calcular los ángulos con acelerometro
   float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
   float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);
 
   //Calcular angulo de rotación con giroscopio y filtro complementario
   ang_x = 0.98 * (ang_x_prev + (gx / 131)*dt) + 0.02 * accel_ang_x;
   ang_y = 0.98 * (ang_y_prev + (gy / 131)*dt) + 0.02 * accel_ang_y;

/*
  if ( ang_x_prev == 0 || ang_y_prev == 0 ) {
    ang_x_prev = ang_x;
    ang_y_prev = ang_y;
  } else {
    if ( ( ang_x + min_variation ) < ang_x_prev || ( ang_x - min_variation ) > ang_x_prev ) {
      ang_x_prev = ang_x;
    }
    
    if ( ( ang_y + min_variation ) < ang_y_prev || ( ang_y - min_variation ) > ang_y_prev ) {
      ang_y_prev = ang_y;
    }
  }
*/

  ang_x_prev = ang_x;
  ang_y_prev = ang_y;
  
}


void setup()
{

  Serial.begin(9600);           // Inicializamos  el puerto serie
  while (!Serial) {
    ;
  }

  if ( DEBUGER) { Serial.println( "Listo!" ) ; }

  /*
   * inicializar wire, esto nos permite leer y escribir en dispositivos i2c
  */
  Wire.begin();
  mpu.initialize();
  if ( DEBUGER ) { Serial.println(mpu.testConnection() ? F("IMU iniciado correctamente") : F("Error al iniciar IMU")); }
  delay( 1000 ) ;  // estabilización de la lectura
 
}
 
void loop()
{

  /*
  * Leer las aceleraciones y velocidades angulares
  */
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
  
  updateFiltered();

  // despreciar la parte decimal de los ángulos
  ang_x = int( ang_x ) ;
  ang_y = int( ang_y ) ;   

  /* 
  *  depurar la lectura del IMU
  */

  if ( DEBUGER ) {
    Serial.print(F("Rotacion en X:  "));
    Serial.print( int( ang_x ) );
    Serial.print(F("\t Rotacion en Y: "));
    Serial.println( int( ang_y ) );
  }  
   
  delay( 10 );

  /*
  * dependiendo de las lecturas en los ejes x e y enviaremos un caracter a través del bluetooth maestro con la
  * siguiente codificación
  * 
  *     s   stop
  *     S   stop
  *     f   forward
  *     F   forward rápido
  *     b   backward
  *     B   backward rápido
  *     r   right
  *     R   right rápido
  *     l   left
  *     L   left rápido
  *     u   up
  *     U   up rápido
  *     d   down
  *     D   down rápido
  *     q   adelante izquierda
  *     Q   adelante izquierda rápido
  *     z   atrás izquierda
  *     Z   atrás izquierda rápido
  *     e   adelante derecha
  *     E   adelante derecha rápìdo
  *     c   atrás derecha
  *     C   atrás derecha ráapido
  *     1   botón 1
  *     2   botón 2
  *     3   botón 3
  *     4   botón 4
  *     
  *     los botones deberán ser hardware agregado al arduino micro controlador
  */

  // parar
  if ( between( ang_x, death_angle * (-1), death_angle ) && between( ang_y, death_angle * (-1), death_angle ) ) {
    if ( DEBUGER ) { Serial.println( "S" ) ; }
  }
    
  // hacia adelante - forward
  if ( ( ang_x < death_angle * (-1) && ang_x > chg_limit * (-1) ) && between( ang_y, death_angle * (-1), death_angle ) ) {
    if ( DEBUGER ) { Serial.println( "f" ) ; }
  }

  // adelante rápido
  if ( ( ang_x > u_limit * (-1) && ang_x <= chg_limit * (-1) ) && between( ang_y, death_angle * (-1), death_angle ) ) {
    if ( DEBUGER ) { Serial.println( "F" ) ; }
  }

  // hacia atrás
  if ( ( ang_x > death_angle && ang_x <= chg_limit ) && between( ang_y, death_angle * (-1), death_angle ) ) {
    if ( DEBUGER ) { Serial.println( "b" ) ; }
  }

  // atrás rápido
  if ( ( ang_x >= chg_limit + 1  && ang_x <= u_limit ) && between( ang_y, death_angle * (-1), death_angle ) ) {
    if ( DEBUGER ) { Serial.println( "B" ) ; }
  }

  // derecha
  if ( between( ang_x, death_angle * (-1), death_angle ) && ( ang_y > death_angle && ang_y <= chg_limit ) ) {
    if ( DEBUGER ) { Serial.println( "r" ) ; }
  }

  // derecha rápido
  if ( between( ang_x, death_angle * (-1), death_angle ) && ( ang_y >= chg_limit + 1 && ang_y <= u_limit) ) {
    if ( DEBUGER ) { Serial.println( "R" ) ; }
  }

  // izquierda
  if ( between( ang_x, death_angle * (-1), death_angle ) && ( ang_y < death_angle * (-1) && ang_y > chg_limit * (-1) ) ) {
    if ( DEBUGER ) { Serial.println( "l" ) ; }
  }

  // izquierda rápido
  if ( between( ang_x, death_angle * (-1), death_angle ) && ( ang_y > u_limit * (-1) && ang_y_prev <= chg_limit * (-1) ) ) {
    if ( DEBUGER ) { Serial.println( "L" ) ; }
  }

  // adelante izquierda lento
  if ( ( ang_x < death_angle * (-1) && ang_x > chg_limit * (-1) ) && ( ang_y < death_angle * (-1) && ang_y > chg_limit * (-1) ) ) {
    if ( DEBUGER ) { Serial.println( "q" ) ; }
  }

  // adelante izquierda rápido
  if ( ( ang_x <= chg_limit * (-1) ) && ( ang_y <= chg_limit * (-1) ) ) {
    if ( DEBUGER ) { Serial.println( "Q" ) ; }
  }

  // atrás izquierda lento
  if ( ( ang_x > death_angle && ang_x <= chg_limit ) && ( ang_y < death_angle * (-1) && ang_y > chg_limit * (-1) ) ) {
    if ( DEBUGER ) { Serial.println( "z" ) ; }
  }
    
  // atrás izquierda rápido
  if ( ( ang_x > chg_limit + 1 ) && ( ang_y <= chg_limit * (-1) ) ) {
    if ( DEBUGER ) { Serial.println( "Z" ) ; }
  }

  // adelante derecha lento
  if ( ( ang_x < death_angle * (-1) && ang_x > chg_limit * (-1) ) && ( ang_y > death_angle && ang_y <= chg_limit ) ) {
    if ( DEBUGER ) { Serial.println( "e" ) ; }
  }

  // adelante derecha rápido
  if ( ( ang_x <= chg_limit * (-1) ) && ( ang_y >= chg_limit + 1 ) ) {
    if ( DEBUGER ) { Serial.println( "E" ) ; }
  }

  // atrás derecha lento
  if ( ( ang_x > death_angle && ang_x <= chg_limit ) && ( ang_y > death_angle && ang_y <= chg_limit ) ) {
    if ( DEBUGER ) { Serial.println( "c" ) ; }
  }

  // atrás derecha rápido
  if ( ( ang_x > chg_limit + 1 ) && ( ang_y > chg_limit + 1 ) ) {
    if ( DEBUGER ) { Serial.println( "C" ) ; }
  }

}

/*
  // parar
  if ( between( ang_x, -2, 2 ) && between( ang_y, -2, 2 ) ) {
    new_cmd = bt_commands[0] ;
  }
    
  // hacia adelante - forward
  if ( ( ang_x < -2 && ang_x > -15 ) && between( ang_y, -2, 2 ) ) {
    new_cmd = "f" ;
  }

  // adelante rápido
  if ( ( ang_x > -40 && ang_x <= -15 ) && between( ang_y, -2, 2 ) ) {
    new_cmd = "F" ;
  }

  // hacia atrás
  if ( ( ang_x > 2 && ang_x <= 15 ) && between( ang_y, -2, 2 ) ) {
    new_cmd = "b" ;
  }

  // atrás rápido
  if ( ( ang_x >= 16  && ang_x <= 40 ) && between( ang_y, -2, 2 ) ) {
    new_cmd = "B" ;
  }

  // derecha
  if ( between( ang_x, -2, 2 ) && ( ang_y > 2 && ang_y <= 15 ) ) {
    new_cmd = "r" ;
  }

  // derecha rápido
  if ( between( ang_x, -2, 2 ) && ( ang_y >= 16 && ang_y <= 40 ) ) {
    new_cmd = "R" ;
  }

  // izquierda
  if ( between( ang_x, -2, 2 ) && ( ang_y < -2 && ang_y > -15 ) ) {
    new_cmd = "l" ;
  }

  // izquierda rápido
  if ( between( ang_x, -2, 2 ) && ( ang_y > -40 && ang_y_prev <= -15 ) ) {
    new_cmd = "L" ;
  }

  // adelante izquierda lento
  if ( ( ang_x < -2 && ang_x > -15 ) && ( ang_y < -2 && ang_y > -15 ) ) {
    new_cmd = "q" ;
  }

  // adelante izquierda rápido
  if ( ( ang_x <= -15 ) && ( ang_y <= -15 ) ) {
    new_cmd = "Q" ;
  }

  // atrás izquierda lento
  if ( ( ang_x > 2 && ang_x <= 15 ) && ( ang_y < -2 && ang_y > -15 ) ) {
    new_cmd = "z" ;
  }
    
  // atrás izquierda rápido
  if ( ( ang_x > 16 ) && ( ang_y <= -15 ) ) {
    new_cmd = "Z" ;
  }

  // adelante derecha lento
  if ( ( ang_x < -2 && ang_x > -15 ) && ( ang_y > 2 && ang_y <= 15 ) ) {
    new_cmd = "e" ;
  }

  // adelante derecha rápido
  if ( ( ang_x <= -15 ) && ( ang_y >= 16 ) ) {
    new_cmd = "E" ;
  }

  // atrás derecha lento
  if ( ( ang_x > 2 && ang_x <= 15 ) && ( ang_y > 2 && ang_y <= 15 ) ) {
    new_cmd = "c" ;
  }

  // atrás derecha rápido
  if ( ( ang_x > 16 ) && ( ang_y > 16 ) ) {
    new_cmd = "C" ;
  }

 */

