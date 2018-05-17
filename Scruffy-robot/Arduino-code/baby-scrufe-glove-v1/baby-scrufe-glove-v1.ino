#include <Servo.h>          // Incluir la librería Servo
#include <SoftwareSerial.h> // para el bluetooth

// Bluetooth y los pines de envío y salida serie
SoftwareSerial BT_slave( 4, 3 ) ;      // 10 RX | 11 TX OJO: en el módulo RX es es TX y viceversa RX->TX TX->RX

Servo turret ;                      // servo de la torreta, giro lateral de 40 a 128 grados
Servo up_sensor ;                   // servo ultrasonido, giro atrás 7º, giro adelante 179º
Servo left_wheel ;                  // servo rueda izquierda
Servo right_wheel ;                 // servo rueda derecha

// sentido de la marcha y giros
#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define TURN_LEFT 5
#define TURN_RIGHT 6
#define LEFT_BACKWARD 7
#define RIGHT_BACKWARD 8
#define LEFT_FORWARD 9
#define RIGHT_FORWARD 10


// velocidad de avance o retroceso en grados del giro de los servos
#define ZERO 0
#define VERYSLOW 3
#define SLOW 4          // 5
#define MEDIUM 5        // 10
#define FAST 6          // 25
#define ULTRAFAST 7     // 180

#define REDUCE_DISTANCE 50
#define SAFETY_DISTANCE 20

#define ARRAY_BT_COMMANDS 25

// depuración global
const boolean DEBUGER = false ;

// pines del sensor de ultrasonido
const int EchoPin = 2 ;             // HC-SR04 pin del eco
const int TriggerPin = 7 ;          // HC-SR04 pin del disparador

// pines del bluetooth
const int bt_enable = 3 ;

// pin del infrarrojo anterior
const int infrared_front_Pin = 8 ;
// pin del infrarrojo posterior
const int infrared_rear_Pin = 4 ;

// claxon = buzzer
const int claxon = 12 ;

// pin para el led de aviso ( las largas )
const int LED = 13 ;

// pin digital conectado a la salida del sensor PIR (sin uso actual)
int Pin_PIR = 0 ;

// ángulos iniciales de los cuatro servos
// torreta, sensor, rueda izquierda, rueda derecha
const int initial_angle[] = { 85, 85, 90, 90 } ;

int turret_angle = 0 ;
int up_sensor_angle = 0 ;
int lw_angle = 0 ;
int rw_angle = 0 ;

// torreta, sensor, rueda izq, rueda dcha.
// const int Pines[] = {10, 9, 5, 6 } ;

// torreta
const int Pin_turret = 10 ;
// sensor ultrasonido
const int Pin_up_sensor = 9 ;
// rueda izquierda
const int Pin_left_wheel = 5 ;
// rueda derecha
const int Pin_right_wheel = 6 ;

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
String bt_commands[ ARRAY_BT_COMMANDS ] = { "S", "f", "F", "b", "B", "r", "R", "l", "L", "q", "Q", "z", "Z", "e", "E", "c", "C", "u", "U", "d", "D", "1", "2", "3", "4" };
String bt_response[ ARRAY_BT_COMMANDS ] = { "Stopping", "forward", "Fast forward", "backward", "Fast Backward", "rigth", "Fast Right", "left", "Fast Left", "Fordward left", "Fast Forward Left", "Backward left", "Fast Backward left", "Forward right", "Fast Forward right", "Backward right", "Fast Backward right", "up", "Fast Up", "down", " Fast Down", "Command 1", "Command 2", "Command 3", "Command 4" };
byte index = 0;                       // índice para recorrer el array de comandos
String bt_read_result = "" ;          // cadena leída desde el bluetooth
int bt_int_result = 0 ;               // comando a ejecutar

String old_cmd = "" ;
String new_cmd = "" ;
char cmd_rcv  ;

// control de velocidad
int current_velocity = 0 ;              // velocidad actual ( SLOW, MEDIUM, FAST )
// sentido de la marcha actual
int current_direction = 0 ;             // sentido de giro actual (0 = STOP)
// guarda la dirección actual en los giros manuales
int cur_to_go = 0 ;
// retardo general
int general_delay = 300 ;
// retardo general para giros de torreta y sensor
int sensor_delay = 350 ;
// retardo posicionamiento servos ruedas delanteras
int servo_read_delay = 100 ;

int cm_derecha = 0 ;      // distancia disponible a la derecha
int cm_izquierda = 0 ;    // distancia disponible a la izquierda
int cm_abajo = 0 ;        // distancia disponible hacia abajo
int cm_arriba = 0 ;       // distancia disponible hacia arriba

boolean stop_moving = false ;     // indica error de lectura de ambos infrarojos

//
//  rutina de setup general
//
void setup() {
  // conectar la salida serie activar el depurado y la entrada del módulo bluetooth
  Serial.begin(9600);

  // pinMode( bt_enable, OUTPUT ) ;         // enable del módulo bluetooth
  // digitalWrite( bt_enable, HIGH ) ;
  // delay (500) ;                          // Espera antes de encender el modulo
  BT_slave.begin(38400);                         // 38400 / 9600 dependiendo si el módulo está en modo AT1 o AT2
  BT_slave.write( "OK\n" ) ;

  turret.attach( Pin_turret ) ;             // Conectar turret al pin definido en pin_turret
  up_sensor.attach( Pin_up_sensor ) ;       // Conectar up_sensor al pin definido en pin_up_sensor
  left_wheel.attach( Pin_left_wheel ) ;     // Conectar left_wheel al pin definido en Pin_left_wheel
  right_wheel.attach( Pin_right_wheel ) ;   // Conectar right_wheel al pin definido en Pin_right_wheel

  pinMode( claxon, OUTPUT );                // buzzer

  pinMode( LED, OUTPUT ) ;                  // pin del LED para mensajes ( las largas )

  pinMode( infrared_front_Pin, INPUT ) ;    // Pin del infrarojo
  pinMode( infrared_rear_Pin, INPUT ) ;     // Pin del infrarojo

  pinMode( TriggerPin, OUTPUT );            // Pin ultrasonido Trigger
  pinMode( EchoPin, INPUT );                // Pin ultrasonido Eco entrante
  digitalWrite( EchoPin, LOW );             // eco del ultrasonido
  digitalWrite( TriggerPin, LOW );          // disparador del ultrasonido

  pinMode( Pin_PIR, INPUT );                // sin uso actual, por implementar

  // servos al valor inicial que indique la variable de posición inicial
  turret.write( initial_angle[ 0 ] ) ;
  up_sensor.write( initial_angle[ 1 ] ) ;
  left_wheel.write( initial_angle[ 2 ] ) ;
  right_wheel.write( initial_angle[ 3 ] ) ;

  // lectura posición servo sensor y torreta
  turret_angle = turret.read();
  up_sensor_angle = up_sensor.read();

  // lectura ángulo de las ruedas paradas
  lw_angle = left_wheel.read();
  rw_angle = right_wheel.read();

  // apagar el LED
  digitalWrite( LED, LOW );

  acoustic_claxon() ;

  // un segundos de espera antes de arrancar, calentando...
  delay( 1000 ) ;

}

//  **************************************************************************************************
//  funciones general del bucle principal
//  **************************************************************************************************

//
//  mirar a la izquierda, búsqueda de objetos, medir distancia
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
//  mirar a la izquierda, búsqueda de objetos, medir distancia
//
int look_at_right() {

  int cm_right = 0 ;

  // OJO, nunca con el sensor inclinado
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
//  mirar al frente, búsqueda de objetos, medir distancia
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
//  mirar hacia abajo, bajar el sensor para estimar la profundidad de algo
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
//  mirar hacia arriba, subir el sensor para estimar la altura de algo
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

int ping(int TriggerPin, int EchoPin) {

  long duration, distanceCm;

repeat:

  digitalWrite( TriggerPin, LOW );  //para generar un pulso limpio ponemos a LOW 4us
  delayMicroseconds( 10 );
  digitalWrite(TriggerPin, HIGH);  //generamos Trigger (disparo) de 10us
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
    if ( DEBUGER ) {
      // Serial.println( "Excedido rango inferior" );
    }

    return 0 ;
  }

  if ( distanceCm > 300 ) {
    if ( DEBUGER ) {
      // Serial.println( "Excededo rango superior" );
    }

    return 999 ;
  }

  return distanceCm;

}

//
//    lectura estado infrarrojo anterior
//
int IR_front_status() {
  // inhabilitado hasta estar conectado, eliminar la línea siguiente cuando lo esté
  return LOW ;

  int IR_front = digitalRead( infrared_front_Pin ) ;
  return IR_front ;
}

//
//    lectura estado infrarrojo posterior
//
int IR_rear_status() {

  // inhabilitado hasta estar conectado, eliminar la línea siguiente cuando lo esté
  return LOW ;

  int IR_rear = digitalRead( infrared_rear_Pin ) ;
  return IR_rear ;
}

//
//    parada de emergencia
//
void emergency_stop() {
  left_wheel.write( initial_angle[ 2 ] ) ;
  right_wheel.write( initial_angle[ 3 ] ) ;

  current_velocity = 0 ;
  current_direction = 0 ;

  delay( servo_read_delay ) ;
}

//
//    ordenes de movimiento completas
//
void move_on( int move_to, int velocity, int tracer ) {

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

      if ( DEBUGER ) {
        Serial.println( " Parado (move_to) " + String( tracer )) ;
      }
    break;

    case FORWARD:
      left_wheel.write( lw_angle + velocity ) ;     // avanza
      right_wheel.write( rw_angle - velocity ) ;    // avanza
      look_at_front() ;

      if ( DEBUGER ) {
        Serial.println( " Adelante " ) ;
      }
      break;

    case BACKWARD:
      left_wheel.write( lw_angle - velocity ) ;     // retrocede
      right_wheel.write( rw_angle + velocity ) ;    // retrocede
      look_at_front() ;

      acoustic_claxon() ;

      if ( DEBUGER ) {
        Serial.println( " Atrás " ) ;
      }
      break;

    case LEFT:
      left_wheel.write( initial_angle[ 2 ] ) ;      // para la rueda para girar y avanzar al mismo tiempo      
      right_wheel.write( rw_angle - velocity ) ;    // avanza        
      
      if ( DEBUGER ) {
        Serial.println( " Giro izquierda " ) ;
      }
    break ;
    
    case RIGHT:
      left_wheel.write( lw_angle + velocity ) ;     // avanza
      right_wheel.write( initial_angle[ 3 ] ) ;     // para la rueda para girar y avanzar al mismo tiempo      
      
      if ( DEBUGER ) {
        Serial.println( " Giro derecha " ) ;
      }
    break ;
    
    case TURN_LEFT:
      left_wheel.write( lw_angle + SLOW ) ;      // para la rueda para girar y avanzar al mismo tiempo      
      right_wheel.write( rw_angle - FAST ) ;    // avanza
      
      if ( DEBUGER ) {
        Serial.println( " Giro izquierda avanzando" ) ;
      }
    break;

    case TURN_RIGHT:
      left_wheel.write( lw_angle + FAST ) ;     // avanza
      right_wheel.write( rw_angle - SLOW ) ;     // para la rueda para girar y avanzar al mismo tiempo      
      
      if ( DEBUGER ) {
        Serial.println( " Giro derecha avanzando" ) ;
      }
    break;

    case LEFT_BACKWARD:
      left_wheel.write( lw_angle - velocity ) ;     // retrocede
      right_wheel.write( initial_angle[ 3 ] ) ;     // para la rueda para girar y avanzar al mismo tiempo              
      
      acoustic_claxon() ;

      if ( DEBUGER ) {
        Serial.println( " Atrás izquierda " ) ;
      }
      break;

    case RIGHT_BACKWARD:
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

  //
  // antes de comenzar a rodar, evaluar el escenario actual, puede que no sea posible
  //

  // sin comandodo BT, anula los valores anteriores si existen
  index = -1 ;
  bt_read_result = "" ;

  // si se reciben datos por BT, leer todos los caracteres disponibles en un string
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
    boolean found = false ;
    for ( index = 0 ; index < ARRAY_BT_COMMANDS ; index++ ) {
      if ( bt_commands[ index ] == cmd_str ) {
        if ( DEBUGER ) {
          Serial.println( " ejecutar: " + bt_response[index] ) ;
        }
        found = true ;
        BT_slave.print( "OK" );   // send response to master
        break ;
      }
    }

    // error de comando
    if ( !found ) {
      BT_slave.println( "ERROR" );   // send response to master
      index = -1 ;
    }
  }


  //
  //  si el comando recibido por BT es válido, ejecutarlo, si no se indicó previamente el modo auto, se puede resolver
  //  1)  ignorando el comando
  //  2)  activando el modo bluetooth automáticamente
  //

  if ( index != -1 ) {
    switch ( index ) {
      case 0 :      // S stop
        move_on( STOP, ZERO, 0 ) ;
      break ;
        
      case 1 :      // f forward
        BT_slave.println( "adelante" ) ;
        if ( current_direction == BACKWARD ) {
          move_on( STOP, ZERO, 1 ) ;
          // delay( sensor_delay ) ;
        }

        look_at_front() ;
        
        move_on( FORWARD, SLOW, 1 ) ;
      break ;

      case 2 :      // F FORWARD
        if ( current_direction == BACKWARD ) {
          move_on( STOP, ZERO, 2 ) ;
        }

        look_at_front() ;

        move_on( FORWARD, FAST, 2 ) ;
      break ;

      case 3 :      // b backward
        if ( current_direction == FORWARD ) {
          move_on( STOP, ZERO, 3 ) ;
        }

        look_down() ;

        move_on( BACKWARD, SLOW, 3 ) ;
      break ;

      case 4 :      // B BACKWARD :
        if ( current_direction == FORWARD ) {
          move_on( STOP, ZERO, 4 ) ;
        }

        look_down() ;

        move_on( BACKWARD, FAST, 4 ) ;
      break ;

      case 5 :      // r right
        look_at_right() ;
        if ( current_direction == FORWARD ) {
          move_on( TURN_RIGHT, SLOW, 5 ) ;
        } else if ( current_direction == BACKWARD ) {
            move_on( RIGHT_BACKWARD, SLOW, 5 ) ;
        } else {
          // move_on( STOP, 0 ) ;
          move_on( RIGHT, SLOW, 5 ) ;
        }
      break ;

      case 6 :      // R RIGHT      
        look_at_right() ;
        if ( current_direction == FORWARD ) {
          move_on( TURN_RIGHT, FAST, 6 ) ;
        } else if ( current_direction == BACKWARD ) {
            move_on( RIGHT_BACKWARD, FAST, 6 ) ;
        } else {
          // move_on( STOP, 0 ) ;
          move_on( RIGHT, FAST, 6 ) ;
        }
      break ;

      case 7 :      // l left      
        look_at_left() ;
        if ( current_direction == FORWARD ) {
          move_on( TURN_LEFT, SLOW, 7 ) ;
        } else if ( current_direction == BACKWARD ) {
            move_on( LEFT_BACKWARD, SLOW, 7 ) ;
        } else {
          // move_on( STOP, 0 ) ;
          move_on( LEFT, SLOW, 7 ) ;
        }
      break ;

      case 8 :      // l LEFT      
        look_at_left() ;
        if ( current_direction == FORWARD ) {
          move_on( TURN_LEFT, FAST, 8 ) ;
        } else if ( current_direction == BACKWARD ) {
            move_on( LEFT_BACKWARD, FAST, 8 ) ;
        } else {
          // move_on( STOP, 0 ) ;
          move_on( LEFT, FAST, 8 ) ;
        }
      break ;

      case 9 :      // q adelante izquierda
        look_at_left() ;
      
        move_on( TURN_LEFT, SLOW, 9 ) ;
      break ;

      case 10 :      // Q ADELANTE IZQUIERDA (rápido)
        look_at_left() ;

        move_on( TURN_LEFT, FAST, 10 ) ;
      break ;

      case 11 :      // z atrás izquierda
        look_at_right() ;

        move_on( LEFT_BACKWARD, SLOW, 11 ) ;
      break ;

      case 12 :       // Z ATRÁS IZQUIERDA (rápido)
        look_at_left() ;

        move_on( LEFT_BACKWARD, FAST, 12 ) ;
      break ;

      case 13 :       // e adelante derecha
        look_at_right() ;

        move_on( RIGHT_FORWARD, SLOW, 14 ) ;
      break ;

      case 14 :       // E ADELANTE DERECHA (rápido)
        look_at_right() ;

        move_on( RIGHT_FORWARD, FAST, 14 ) ;
      break ;

      case 15 :       // c atraś derecha
        look_at_right() ;

        move_on( LEFT_FORWARD, SLOW, 15 ) ;
      break ;

      case 16 :       // C ATRÁS DERECHA (rápido)
        look_at_right() ;
        
        move_on( LEFT_FORWARD, FAST, 16 ) ;        
      break ;
    }
  }       //  if ( index != -1 ) {

  //
  //  status del lector de infrarojos anterior y posterior, si el modo BT está activado se dan dos escenarios:
  //  1)  que el usuario puede accidentar el vehículo
  //  2)  que los sensores supervisen y decidan las maniobras básicas: proximidad y falta de suelo
  //
  IR_front_value = IR_front_status() ;
  IR_rear_value = IR_rear_status() ;

  if ( IR_front_value == HIGH ) {
    // no hay suelo, parada de emergencia
    if ( DEBUGER ) {
      // Serial.println( "2. ¡No detectado suelo delante! " ) ;
    }

    emergency_stop() ;
  }

  // lectura del sensor de ultrasonido delantero, avance hacia adelante
  int cm = ping( TriggerPin, EchoPin ) ;

  if (DEBUGER ) {
    // Serial.println( "1. Distancia frontal: " + String( cm ) + " suelo: " + String( IR_front_value ) ) ;
  }

  // ambos infrarrojos no dan señal
  if ( stop_moving ) {
    if ( IR_front_value == HIGH && IR_rear_value == HIGH ) {
      goto comienzo ;
    } else {
      stop_moving = false ;
    }
  }

  //
  // parar si hay un objeto cerca, también cuando se tapa el sensor u ocurre alguna anomalía de lectura, el sensor es bastante inestable y
  // tiene muchas lecturas espúreas, incrementar el tiempo de lectura para evitar muchas lecturas contínuas
  //
  //  Si el control manual por BT está activado, existen dos escenario:
  //  1)  el usuario impacta contra el obstáculo
  //  2)  supervisamos la maniobra para impedirlo
  //
  if ( cm <= SAFETY_DISTANCE ) {
    emergency_stop() ;
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

