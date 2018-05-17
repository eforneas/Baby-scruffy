0t 

//GND - GND
//VCC - VCC
//SDA - Pin A4
//SCL - Pin A5

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#include <SoftwareSerial.h>   // Incluimos la librería  SoftwareSerial  

SoftwareSerial BT_master( 8, 9 );      // Definimos los pines RX y TX del Arduino conectados al Bluetooth
SoftwareSerial BT_slave( 6, 7 );         // Definimos los pines RX y TX del Arduino conectados al Bluetooth

/*

    if (0 <= x && x <= 3) // between
    if (y < 0 || 10 < y) // outside
*/
#define between(x, a, b)  (((a) <= (x)) && ((x) <= (b)))
#define outside(x, a, b)  (((x) < (a)) || ((b) < (x)))
#define DEBUGER true
#define ARRAY_BT_COMMANDS 25

// variación mínima para un cambio de valor de lectura
const float min_variation = 0.50 ;

const int mpuAddress = 0x68;  // Puede ser 0x68 o 0x69
MPU6050 mpu(mpuAddress);

int ax, ay, az;
int gx, gy, gz;

long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

// variables for bluetooth
const byte numChars = 254;
char receivedChars[numChars]; // an array to store the received data
boolean newData = false ;
char rc ;

String old_cmd = "" ;
String new_cmd = "" ;
char cmd_rcv  ;
byte index ;

String bt_commands[ ARRAY_BT_COMMANDS ] = { "S", "f", "F", "b", "B", "r", "R", "l", "L", "q", "Q", "z", "Z", "e", "E", "c", "C", "u", "U", "d", "D", "1", "2", "3", "4" };
String bt_response[ ARRAY_BT_COMMANDS ] = { "->S", "->f", "->F", "->b", "->B", "->r", "->R", "->l", "->L", "->q", "->Q", "->z", "->Z", "->e", "->E", "->c", "->C", "->u", "->U", "->d", "->D", "->1", "->2", "->3", "->4" };
//String bt_response[ ARRAY_BT_COMMANDS ] = { "Stopping", "forward", "Fast forward", "backward", "Fast Backward", "rigth", "Fast Right", "left", "Fast Left", "Fordward left", "Fast Forward Left", "Backward left", "Fast Backward left", "Forward right", "Fast Forward right", "Backward right", "Fast Backward right", "up", "Fast Up", "down", " Fast Down", "Command 1", "Command 2", "Command 3", "Command 4" };

// la constante limita el ángulo de cambio de comando fF, bB, lL, rR, etc.
const int chg_limit = 23 ;
// ángulo a partir del cual no genera comando
const int u_limit = 55 ;
// ángulo (máximo y mínimo) que se considera "plano" en parada (stop) --> no acción
const int death_angle = 8 ;


void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
 
   while (Serial.available() > 0 && newData == false) {
     rc = Serial.read();

     if (rc != endMarker) {
       receivedChars[ndx] = rc;
       ndx++;
       if (ndx >= numChars) {
         ndx = numChars - 1;
       }
      } else {
        receivedChars[ndx] = '\0'; // terminate the string
        ndx = 0;
        newData = true;
      }
    }
}

void showNewData() {
  if (newData == true) {

    Serial.println() ;
    Serial.print( "Sendind data: " );
    Serial.println( receivedChars );

    BT_master.print( receivedChars ) ;  // print in master...
    BT_slave.listen() ;                 // ... and listen in slave

    newData = false;

  }
}


void updateFiltered()
{
  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  //Calcular los ángulos con acelerometro
  float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
  float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);

  //Calcular angulo de rotación con giroscopio y filtro complementario
  ang_x = 0.98 * (ang_x_prev + (gx / 131) * dt) + 0.02 * accel_ang_x;
  ang_y = 0.98 * (ang_y_prev + (gy / 131) * dt) + 0.02 * accel_ang_y;

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

  ang_x_prev = int( ang_x ) ;
  ang_y_prev = int( ang_y ) ;

}


void setup() {

  /*
  pinMode(7, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(6, INPUT);
  pinMode(8, INPUT);
  */
  BT_master.begin( 38400 ) ;  //
  BT_slave.begin( 38400 ) ;   // last open, last listen

  delay( 100 ) ;

  Wire.begin();
  mpu.initialize();
  if ( DEBUGER ) {
    Serial.println(mpu.testConnection() ? F("IMU iniciado correctamente") : F("Error al iniciar IMU")) ;
  }
  delay( 1000 ) ;  // estabilización de la lectura

  Serial.begin( 9600 );

  while (!Serial) {
    ;
  }

  if ( DEBUGER ) {
    Serial.println( "Bluetooth Glove ready!" ) ;
  }
}

void loop() {
  // Leer las aceleraciones y velocidades angulares
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  updateFiltered();

  // despreciar la parte decimal de los ángulos
  ang_x = int( ang_x ) ;
  ang_y = int( ang_y ) ;

  delay( 10 );

  /*
     valores de lectura de los ángulos de rotación
  */
  /*
    Serial.print(F("Rotacion en X:  "));
    Serial.print( int( ang_x ) );
    Serial.print(F("\t Rotacion en Y: "));
    Serial.println( int( ang_y ) );
  */

  //     new_cmd = bt_commands[0] ;

  // parar
  if ( between( ang_x, death_angle * (-1), death_angle ) && between( ang_y, death_angle * (-1), death_angle ) ) {
    new_cmd = bt_commands[0] ;
  }
    
  // hacia adelante - forward
  if ( ( ang_x < death_angle * (-1) && ang_x > chg_limit * (-1) ) && between( ang_y, death_angle * (-1), death_angle ) ) {
    new_cmd = bt_commands[1] ;
  }

  // adelante rápido
  if ( ( ang_x > u_limit * (-1) && ang_x <= chg_limit * (-1) ) && between( ang_y, death_angle * (-1), death_angle ) ) {
    new_cmd = bt_commands[2] ;
  }

  // hacia atrás
  if ( ( ang_x > death_angle && ang_x <= chg_limit ) && between( ang_y, death_angle * (-1), death_angle ) ) {
    new_cmd = bt_commands[3] ;
  }

  // atrás rápido
  if ( ( ang_x >= chg_limit + 1  && ang_x <= u_limit ) && between( ang_y, death_angle * (-1), death_angle ) ) {
    new_cmd = bt_commands[4] ;
  }

  // derecha
  if ( between( ang_x, death_angle * (-1), death_angle ) && ( ang_y > death_angle && ang_y <= chg_limit ) ) {
    new_cmd = bt_commands[5] ;
  }

  // derecha rápido
  if ( between( ang_x, death_angle * (-1), death_angle ) && ( ang_y >= chg_limit + 1 && ang_y <= u_limit) ) {
    new_cmd = bt_commands[6] ;
  }

  // izquierda
  if ( between( ang_x, death_angle * (-1), death_angle ) && ( ang_y < death_angle * (-1) && ang_y > chg_limit * (-1) ) ) {
    new_cmd = bt_commands[7] ;
  }

  // izquierda rápido
  if ( between( ang_x, death_angle * (-1), death_angle ) && ( ang_y > u_limit * (-1) && ang_y_prev <= chg_limit * (-1) ) ) {
    new_cmd = bt_commands[8] ;
  }

  // adelante izquierda lento
  if ( ( ang_x < death_angle * (-1) && ang_x > chg_limit * (-1) ) && ( ang_y < death_angle * (-1) && ang_y > chg_limit * (-1) ) ) {
    new_cmd = bt_commands[9] ;
  }

  // adelante izquierda rápido
  if ( ( ang_x <= chg_limit * (-1) ) && ( ang_y <= chg_limit * (-1) ) ) {
    new_cmd = bt_commands[10] ;
  }

  // atrás izquierda lento
  if ( ( ang_x > death_angle && ang_x <= chg_limit ) && ( ang_y < death_angle * (-1) && ang_y > chg_limit * (-1) ) ) {
    new_cmd = bt_commands[11] ;
  }
    
  // atrás izquierda rápido
  if ( ( ang_x > chg_limit + 1 ) && ( ang_y <= chg_limit * (-1) ) ) {
    new_cmd = bt_commands[12] ;
  }

  // adelante derecha lento
  if ( ( ang_x < death_angle * (-1) && ang_x > chg_limit * (-1) ) && ( ang_y > death_angle && ang_y <= chg_limit ) ) {
    new_cmd = bt_commands[13] ;
  }

  // adelante derecha rápido
  if ( ( ang_x <= chg_limit * (-1) ) && ( ang_y >= chg_limit + 1 ) ) {
    new_cmd = bt_commands[14] ;
  }

  // atrás derecha lento
  if ( ( ang_x > death_angle && ang_x <= chg_limit ) && ( ang_y > death_angle && ang_y <= chg_limit ) ) {
    new_cmd = bt_commands[15] ;
  }

  // atrás derecha rápido
  if ( ( ang_x > chg_limit + 1 ) && ( ang_y > chg_limit + 1 ) ) {
    new_cmd = bt_commands[16] ;
  }
  
  // send trought bluetooth master
  if ( new_cmd != old_cmd ) {
    // para no enviar comandos constantemente
    old_cmd = new_cmd ;
    if ( DEBUGER ) { Serial.print( "Nuevo comando: " + new_cmd + " --> ") ; }
    for ( index = 0 ; index < ARRAY_BT_COMMANDS ; index++ ) {
      if ( bt_commands[ index ] == new_cmd ) {
        if ( DEBUGER ) { Serial.println( "acción: " + bt_response[index] ) ; }
        
        char envio[2] ;
        new_cmd.toCharArray( envio, 2 ) ;
        // master send to slave the new command, send as 1 char + \0
        BT_master.write( envio ) ;
        BT_slave.listen() ;
      }
    }

  }

  if ( BT_slave.available() > 0 ) {
    while ( BT_slave.available() > 0 ) {
      cmd_rcv = BT_slave.read();
    }

    if ( DEBUGER ) { 
      Serial.print( "Comando recibido: " ) ; 
      Serial.print( cmd_rcv ) ; 
    }

    // received as char and transform to string for compare
    String cmd_str ;
    cmd_str = String( cmd_rcv ) ;
    // test the command and take the action
    boolean found = false ;
    for ( index = 0 ; index < ARRAY_BT_COMMANDS ; index++ ) {
      if ( bt_commands[ index ] == cmd_str ) {
        if ( DEBUGER ) {
          Serial.println( " ejecutar: " + bt_response[index] ) ;
        }
        found = true ;
        BT_slave.print( "OK" );   // send response to master
        BT_master.listen() ;      // master is listening now
      }
    }

    // error de comando
    if ( !found ) {
      BT_slave.println( );   // send response to master
      BT_slave.println( "ERROR" );   // send response to master
      BT_master.listen() ;      // master is listening now      
    }
    
  }

  if ( BT_master.available() > 0 ) {
    while ( BT_master.available() > 0 ) {
      rc = BT_master.read();
      if ( DEBUGER ) {
        Serial.print( rc ) ;
      }
    }
    
    if ( DEBUGER ) {
      Serial.println() ;
    }
    
    BT_slave.listen() ; // change port to listen
  }

  // capture and send serial from master to slave
  recvWithEndMarker();
  showNewData();

}
