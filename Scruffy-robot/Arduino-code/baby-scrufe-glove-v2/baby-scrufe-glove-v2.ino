#include <Servo.h>          // Incluir la librería Servo
#include <SoftwareSerial.h> // para el bluetooth

// Bluetooth y los pines de envío y salida serie
SoftwareSerial BT_slave( 4, 3 ) ;      // 10 RX | 11 TX OJO: en el módulo RX es es TX y viceversa RX->TX TX->RX

Servo turret ;                      // servo de la torreta, giro lateral de 40 a 128 grados
Servo up_sensor ;                   // servo ultrasonido, giro atrás 7º, giro adelante 179º
Servo left_wheel ;                  // servo rueda izquierda
Servo right_wheel ;                 // servo rueda derecha

// sentido de la marcha y giros
#define STOP 0            // parar
#define FORWARD 1         // marcha adelante
#define BACKWARD 2        // marcha atrás
#define LEFT 3            // desde parado
#define RIGHT 4           // desde parado
#define TURN_LEFT 5
#define TURN_RIGHT 6
#define LEFT_BACKWARD 7   // en movimiento
#define RIGHT_BACKWARD 8  // en movimiento
#define LEFT_FORWARD 9    // en movimiento
#define RIGHT_FORWARD 10  // en movimiento


// velocidad de avance o retroceso en grados del giro de los servos
#define ZERO 0
#define SLOW 4          // 4
#define FAST 8          // 6

#define REDUCE_DISTANCE 30
#define SAFETY_DISTANCE 15

// depuración global
const boolean DEBUGER = false ;

// pines del sensor de ultrasonido
const int EchoPin = 2 ;             // HC-SR04 pin del eco
const int TriggerPin = 7 ;          // HC-SR04 pin del disparador

// pines del bluetooth

// pin del infrarrojo anterior
const int infrared_front_Pin = 8 ;
// pin del infrarrojo posterior
const int infrared_rear_Pin = 4 ;

// claxon = buzzer
const int claxon = 12 ;

// pin para el led de aviso ( las largas )
const int LED = 13 ;

// pin digital conectado a la salida del sensor PIR (sin uso actual)
// ángulos iniciales de los cuatro servos
// torreta, sensor, rueda izquierda, rueda derecha
const int initial_angle[] = { 85, 80, 90, 90 } ;

int turret_angle = 0 ;
int up_sensor_angle = 0 ;
int lw_angle = 0 ;
int rw_angle = 0 ;

// pines de la torreta, sensor, rueda izq, rueda dcha.
const int Pines[] = {10, 9, 5, 6 } ;

// valores máximos y mínimos para el servo de turno
int min_turret_angle_value =  35 ;      // giro a la izquierda de la torreta
int max_turret_angle_value = 145 ;      // giro a la derecha de la torreta

int min_up_sensor_angle_value =  35 ;   // ángulo máximo sensor ultrasonido hacia atrás
int max_up_sensor_angle_value = 140 ;   // ángulo máximo sensor ultrasonido hacia adelante

// valor de lectura del infrarrojo anterior
int IR_front_value = 0 ;
// valor de lectura del infrarrojo posterior
int IR_rear_value = 0 ;

// comandos reconocidos desde la conexión bluetooth
String bt_commands = "SfFbBrRlLqQzZeEcCuUdD1234" ;
byte index = 0;                       // índice para recorrer el array de comandos

char cmd_rcv  ;

// control de velocidad
int current_velocity = 0 ;              // current speed ( SLOW, FAST )

// sentido de la marcha actual
int current_direction = 0 ;             // current direction (0 = STOP)

// retardo general
int general_delay = 150 ;

// retardo general para giros de torreta y sensor
int sensor_delay = 150 ;

// retardo posicionamiento servos ruedas delanteras
int servo_read_delay = 200 ;

//
//  general setup
//
void setup() {
  // conectar la salida serie activar el depurado y la entrada del módulo bluetooth
  Serial.begin(9600);

  BT_slave.begin(38400);                         // 38400 / 9600 dependiendo si el módulo está en modo AT1 o AT2
  BT_slave.write( "OK\n" ) ;
  delay( 200 ) ;

  turret.attach( Pines[0] ) ;             // Conectar turret al pin definido en pin_turret
  up_sensor.attach( Pines[1] ) ;       // Conectar up_sensor al pin definido en pin_up_sensor
  left_wheel.attach( Pines[2] ) ;     // Conectar left_wheel al pin definido en Pin_left_wheel
  right_wheel.attach( Pines[3] ) ;   // Conectar right_wheel al pin definido en Pin_right_wheel

  pinMode( claxon, OUTPUT );                // buzzer

  pinMode( LED, OUTPUT ) ;                  // wanrning LED

  pinMode( infrared_front_Pin, INPUT ) ;    // front IR
  pinMode( infrared_rear_Pin, INPUT ) ;     // rear IR

  pinMode( TriggerPin, OUTPUT );            // Trigger
  pinMode( EchoPin, INPUT );                // Eco
  digitalWrite( EchoPin, LOW );             // init
  digitalWrite( TriggerPin, LOW );          // init

  // servos initial position
  turret.write( initial_angle[ 0 ] ) ;
  up_sensor.write( initial_angle[ 1 ] ) ;
  left_wheel.write( initial_angle[ 2 ] ) ;
  right_wheel.write( initial_angle[ 3 ] ) ;

  // turret servos position
  turret_angle = turret.read();
  up_sensor_angle = up_sensor.read();

  // wheels position
  lw_angle = left_wheel.read();
  rw_angle = right_wheel.read();

  // LED off
  digitalWrite( LED, LOW );

  // delay to init
  delay( 1000 ) ;

  // finish sound to indicate all are ready
  acoustic_claxon() ;

}

//  **************************************************************************************************
//  basic & general functions
//  **************************************************************************************************

//
//  measeure left distance
//
int look_at_left() {

  int cm_left = 0 ;

  // OJO, nunca con el sensor inclinado
  if ( up_sensor.read() != initial_angle[ 1 ] ) {
    up_sensor.write( initial_angle[ 1 ] ) ;
    delay( sensor_delay ) ;
  }

  turret.write( max_turret_angle_value ) ;
  delay( sensor_delay ) ;
  for ( int contador = 0 ; contador <= 4 ; contador++ ) {
    cm_left = max( cm_left, ping( TriggerPin, EchoPin ) ) ;
  }
  return cm_left ;
}

//
//  measeure left distance
//
int look_at_right() {

  int cm_right = 0 ;

  // Warning at US position
  if ( up_sensor.read() != initial_angle[ 1 ] ) {
    up_sensor.write( initial_angle[ 1 ] ) ;
    delay( sensor_delay ) ;
  }

  turret.write( min_turret_angle_value ) ;
  delay( sensor_delay ) ;
  for ( int contador = 0 ; contador <= 4 ; contador++ ) {
    cm_right = max( cm_right, ping( TriggerPin, EchoPin ) ) ;
  }
  return cm_right ;
}

//
//  measeure front distance
//
int look_at_front() {

  int cm_front = 0 ;

  turret.write( initial_angle[ 0 ] ) ;
  delay( sensor_delay ) ;
  up_sensor.write( initial_angle[ 1 ] ) ;
  delay( sensor_delay ) ;
  for ( int contador = 0 ; contador <= 4 ; contador++ ) {
    cm_front = max( cm_front , ping( TriggerPin, EchoPin ) ) ;
  }
  return cm_front ;
}

//
//  measeure down distance
//
int look_down() {

  int cm_down = 0 ;

  up_sensor.write( max_up_sensor_angle_value ) ;
  delay( sensor_delay ) ;
  for ( int contador = 0 ; contador <= 4 ; contador++ ) {
    cm_down = max( cm_down , ping( TriggerPin, EchoPin ) ) ;
  }
  return cm_down ;
}

//
//  measeure up distance
//
int look_up() {

  int cm_up = 0 ;

  up_sensor.write( min_up_sensor_angle_value ) ;
  delay( sensor_delay ) ;
  for ( int contador = 0 ; contador <= 4 ; contador++ ) {
    cm_up = max( cm_up , ping( TriggerPin, EchoPin ) ) ;
  }
  return cm_up ;
}

// US trigger in metric
int ping(int TriggerPin, int EchoPin) {

  long duration, distanceCm;

repeat:

  digitalWrite( TriggerPin, LOW );  //para generar un pulso limpio ponemos a LOW 4us
  delayMicroseconds( 10 );
  digitalWrite(TriggerPin, HIGH);   //generamos Trigger (disparo) de 10us
  delayMicroseconds( 10 );
  digitalWrite( TriggerPin, LOW );

  duration = pulseIn( EchoPin, HIGH );  //medimos el tiempo entre pulsos, en microsegundos

  // depuramos la duración del eco recogido
  if ( DEBUGER ) {
    // Serial.print( "Duración: " + String( duration ) + " "  );
  }

  // duración muy corta, sensor tapado u objeto muy cercano
  if ( duration <= 200 ) {
    return 0 ;
  }

  // medida excesiva, sin alcance, indeterminado, supera el rango máximo del sensor
  if ( duration > 18000 ) {
    return 999 ;
  }

  // distanceCm = duration * 10 / 292 / 2;   //convertimos a distancia, en cm
  // distanceCm = ( duration / 29.2 ) / 2;   //convertimos a distancia, en cm 29,2 microsegundos = 1 centímetro

  distanceCm = int( 0.017 * duration );

  // distancia no válida, evita lecturas mayores de 200 cm, aunque parece que alcanza hasta 300 cm
  // también para valores por debajo de 0
  if ( distanceCm <= 3 ) {
    return 0 ;
  }

  if ( distanceCm > 300 ) {
    return 999 ;
  }

  return distanceCm;

}

//
//    front IR
//
int IR_front_status() {
  int IR_front = digitalRead( infrared_front_Pin ) ;
  return IR_front ;
}

//
//    rear IR
//
int IR_rear_status() {

  // inhabilitado hasta estar conectado, eliminar la línea siguiente cuando lo esté
  return LOW ;

  int IR_rear = digitalRead( infrared_rear_Pin ) ;
  return IR_rear ;
}

//
//    emergency stopping
//
void emergency_stop() {
  left_wheel.write( initial_angle[ 2 ] ) ;
  right_wheel.write( initial_angle[ 3 ] ) ;

  current_velocity = 0 ;
  current_direction = 0 ;

  delay( servo_read_delay ) ;
}

//
//    the mean movement
//
void move_on( int move_to, int velocity ) {

  // mover hacia adelante, controlar el infrarojo delantero
  if ( move_to == FORWARD || move_to == TURN_LEFT || move_to == TURN_RIGHT || move_to == LEFT || move_to == RIGHT || move_to == LEFT_FORWARD || move_to == RIGHT_FORWARD ) {
    int ir_front = IR_front_status() ;
    if ( ir_front == HIGH ) {
      emergency_stop() ;
      return ;
    }
  }

  // movimiento hacia atrás, controlar el infrarojo trasero
  if ( move_to == BACKWARD || move_to == LEFT_BACKWARD || move_to == RIGHT_BACKWARD ) {
    int ir_rear = IR_rear_status() ;
    if ( ir_rear == HIGH ) {
      emergency_stop() ;
      return ;
    }
  }

  // read sultrasound sensor
  int cm = ping( TriggerPin, EchoPin ) ;

  // no safe distance for FAST speed
  if ( cm <= REDUCE_DISTANCE && velocity == FAST ) {
    velocity = LOW ;
  }
  
  // lectura posición actual de ambos servos
  lw_angle = left_wheel.read();
  rw_angle = right_wheel.read();

  if ( DEBUGER ) {
    // Serial.print ( " LeftW: " + String( lw_angle ) + "º RightW: " + String( rw_angle ) );
  }

  switch ( move_to ) {
    case STOP:
      left_wheel.write( initial_angle[ 2 ] ) ;
      right_wheel.write( initial_angle[ 3 ] ) ;
      // look_at_front() ;

      if ( DEBUGER ) {
        Serial.println( " Parado (move_to) " ) ;
      }
    break;

    case FORWARD:
      // look_at_front() ;
    
      if ( current_velocity == FAST ) {
        left_wheel.write( initial_angle[ 2 ] ) ;     // avanza
        right_wheel.write( initial_angle[ 3 ] ) ;    // avanza
        lw_angle = left_wheel.read();
        rw_angle = right_wheel.read();
      }
      left_wheel.write( lw_angle + velocity ) ;     // avanza
      right_wheel.write( rw_angle - velocity ) ;    // avanza

      if ( DEBUGER ) {
        Serial.println( " Adelante " ) ;
      }
    break;

    case BACKWARD:
      // look_at_front() ;

      if ( current_velocity == FAST ) {
        left_wheel.write( initial_angle[ 2 ] ) ;     // avanza
        right_wheel.write( initial_angle[ 3 ] ) ;    // avanza
        delay( servo_read_delay ) ;
        lw_angle = left_wheel.read();
        rw_angle = right_wheel.read();
      }
      left_wheel.write( lw_angle - velocity ) ;     // retrocede
      right_wheel.write( rw_angle + velocity ) ;    // retrocede

      acoustic_claxon() ;

      if ( DEBUGER ) {
        Serial.println( " Atrás " ) ;
      }
    break;

    case LEFT:
      // look_at_left() ;

      if ( current_velocity == FAST ) {
        left_wheel.write( initial_angle[ 2 ] ) ;     // ángulo inicial
        right_wheel.write( initial_angle[ 3 ] ) ;    // ángulo inicial
        delay( servo_read_delay ) ;
        lw_angle = left_wheel.read();
        rw_angle = right_wheel.read();
      }
      // left_wheel.write( initial_angle[ 2 ] ) ;      // ángulo inicial
      left_wheel.write( lw_angle - velocity ) ;     // retrocede      
      right_wheel.write( rw_angle - velocity ) ;    // avanza

      if ( DEBUGER ) {
        Serial.println( " Giro izquierda " ) ;
      }
    break ;

    case RIGHT:
      // look_at_right() ;
      
      if ( current_velocity == FAST ) {
        left_wheel.write( initial_angle[ 2 ] ) ;     // ángulo inicial
        right_wheel.write( initial_angle[ 3 ] ) ;    // ángulo inicial
        delay( servo_read_delay ) ;
        lw_angle = left_wheel.read();
        rw_angle = right_wheel.read();
      }
      left_wheel.write( lw_angle + velocity ) ;     // avanza
      // right_wheel.write( initial_angle[ 3 ] ) ;     // ángulo inicial
      right_wheel.write( rw_angle + velocity ) ;    // retrocede

      if ( DEBUGER ) {
        Serial.println( " Giro derecha " ) ;
      }
    break ;

    case LEFT_FORWARD:
      // look_at_left() ;

      left_wheel.write( initial_angle[ 2 ] ) ;      // para la rueda para girar y avanzar al mismo tiempo
      lw_angle = left_wheel.read();
      left_wheel.write( lw_angle + SLOW ) ;      // para la rueda para girar y avanzar al mismo tiempo
      right_wheel.write( rw_angle - FAST ) ;    // avanza

      if ( DEBUGER ) {
        Serial.println( " Giro izquierda avanzando " ) ;
      }
    break;

    case RIGHT_FORWARD:
      // look_at_right() ;
      
      left_wheel.write( lw_angle + FAST ) ;     // avanza
      right_wheel.write( initial_angle[ 3 ] ) ;     // para la rueda para girar y avanzar al mismo tiempo
      rw_angle = right_wheel.read();
      right_wheel.write( rw_angle - SLOW ) ;     // para la rueda para girar y avanzar al mismo tiempo

      if ( DEBUGER ) {
        Serial.println( " Giro derecha avanzando " ) ;
      }
    break;

    case LEFT_BACKWARD:
      // look_at_left() ;
      
      left_wheel.write( lw_angle - velocity ) ;     // retrocede
      right_wheel.write( initial_angle[ 3 ] ) ;     // para la rueda para girar y avanzar al mismo tiempo

      acoustic_claxon() ;

      if ( DEBUGER ) {
        Serial.println( " Atrás izquierda " ) ;
      }
    break;

    case RIGHT_BACKWARD:
      // look_at_right() ;

      left_wheel.write( initial_angle[ 2 ] ) ;      // para la rueda para girar y avanzar al mismo tiempo
      right_wheel.write( rw_angle + velocity ) ;    // retrocede

      acoustic_claxon() ;

      if ( DEBUGER ) {
        Serial.println( " Atrás derecha " ) ;
      }
    break;

  }

  // guardamos la velocidad actual
  current_velocity = velocity ;
  //  guardamos la dirección actual
  current_direction = move_to ;

  // esperar hasta que el motor tome la nueva posición
  delay( servo_read_delay ) ;

}

//
//  señales acústicas y luminosas
//
void acoustic_claxon() {

  // señal acústica de marcha atrás
  tone( claxon, 1000 ) ;
  delay( 200 ) ;
  noTone( claxon ) ;

}


void loop() {

comienzo:

  // command no found, default
  index = -1 ;

  // receive command in 1 char
  if ( BT_slave.available() ) {
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
    index = bt_commands.indexOf( cmd_str ) ;
/*
    boolean found = false ;
    for ( index = 0 ; index < ARRAY_BT_COMMANDS ; index++ ) {
      if ( bt_commands[ index ] == cmd_str ) {
        found = true ;
        if ( DEBUGER ) {
          // Serial.println( " ejecutar: " + bt_response[index] ) ;
          Serial.println( " ejecutar: " + bt_commands[index] ) ;
          BT_slave.print( "OK" );   // send response to master
        }
        break ;
      }
    }

    // error de comando
    if ( !found ) {
      if ( DEBUGER ) {
        BT_slave.println( "ERROR" );   // send response to master
      }
      index = -1 ;
    }
*/
  }

  // read sultrasound sensor
  int cm = ping( TriggerPin, EchoPin ) ;

  //
  //  IR status, rear and front sensors
  //
  IR_front_value = IR_front_status() ;
  IR_rear_value = IR_rear_status() ;

  // no floor os distance no secure
  if ( IR_front_value == HIGH || IR_rear_value == HIGH || cm <= SAFETY_DISTANCE ) {
    emergency_stop() ;
  }

  //
  //  si el comando recibido por BT es válido, ejecutarlo, si no se indicó previamente el modo auto, se puede resolver
  //  1)  ignorando el comando
  //  2)  activando el modo bluetooth automáticamente
  //

  if ( index != -1 ) {
    switch ( index ) {
      case 0 :      // S stop
        move_on( STOP, ZERO ) ;
      break ;

      case 1 :      // f forward
        BT_slave.println( "adelante" ) ;
        if ( current_direction == BACKWARD ) {
          move_on( STOP, ZERO ) ;
        }
        move_on( FORWARD, SLOW ) ;
      break ;

      case 2 :      // F FORWARD
        if ( current_direction == BACKWARD ) {
          move_on( STOP, ZERO ) ;
        }
        move_on( FORWARD, FAST ) ;
      break ;

      case 3 :      // b backward
        if ( current_direction == FORWARD ) {
          move_on( STOP, ZERO ) ;
        }
        move_on( BACKWARD, SLOW ) ;
      break ;

      case 4 :      // B BACKWARD :
        if ( current_direction == FORWARD ) {
          move_on( STOP, ZERO ) ;
        }
        move_on( BACKWARD, FAST ) ;
      break ;

      case 5 :      // r right
        if ( current_direction == FORWARD ) {
          move_on( RIGHT_FORWARD, SLOW ) ;
        } else if ( current_direction == BACKWARD ) {
          move_on( RIGHT_BACKWARD, SLOW ) ;
        } else {
          move_on( RIGHT, SLOW ) ;
        }
      break ;

      case 6 :      // R RIGHT (rápido)
        if ( current_direction == FORWARD ) {
          move_on( RIGHT_FORWARD, FAST ) ;
        } else if ( current_direction == BACKWARD ) {
          move_on( RIGHT_BACKWARD, FAST ) ;
        } else {
          move_on( RIGHT, FAST ) ;
        }
      break ;

      case 7 :      // l left
        if ( current_direction == FORWARD ) {
          move_on( LEFT_FORWARD, SLOW ) ;
        } else if ( current_direction == BACKWARD ) {
          move_on( LEFT_BACKWARD, SLOW ) ;
        } else {
          move_on( LEFT, SLOW ) ;
        }
      break ;

      case 8 :      // L LEFT (rápido)
        if ( current_direction == FORWARD ) {
          move_on( LEFT_FORWARD, FAST ) ;
        } else if ( current_direction == BACKWARD ) {
          move_on( LEFT_BACKWARD, FAST ) ;
        } else {
          move_on( LEFT, FAST ) ;
        }
      break ;

      case 9 :      // q adelante izquierda
        move_on( LEFT_FORWARD, SLOW ) ;
      break ;

      case 10 :      // Q ADELANTE IZQUIERDA (rápido)
        move_on( LEFT_FORWARD, FAST ) ;
      break ;

      case 11 :      // z atrás izquierda
        move_on( LEFT_BACKWARD, SLOW ) ;
      break ;

      case 12 :       // Z ATRÁS IZQUIERDA (rápido)
        move_on( LEFT_BACKWARD, FAST ) ;
      break ;

      case 13 :       // e adelante derecha
        move_on( RIGHT_FORWARD, SLOW ) ;
      break ;

      case 14 :       // E ADELANTE DERECHA (rápido)
        move_on( RIGHT_FORWARD, FAST ) ;
      break ;

      case 15 :       // c atraś derecha
        move_on( RIGHT_FORWARD, SLOW ) ;
      break ;

      case 16 :       // C ATRÁS DERECHA (rápido)
        move_on( RIGHT_FORWARD, FAST ) ;
      break ;
    }
  }       //  if ( index != -1 ) {

  // in short distance not FAST speed aloweed
  if ( cm <= SAFETY_DISTANCE && current_velocity == FAST) {
    current_velocity = LOW ;
  }

  if ( DEBUGER ) {
    // Serial.println( "8. Dirección: " + String( current_direction ) + " velocidad " + String( current_velocity ) ) ;
    // Serial.print ( " - Turret: " + String( turret_angle ) + "º Sensor: " + String( up_sensor_angle ) );
    // Serial.print ( " LeftW: " + String( lw_angle ) + "º RightW: " + String( rw_angle ) );
    // Serial.print ( " - objeto a " + String(cm) + " cm " );
    // Serial.println();
  }

  delay( general_delay ) ;

}

