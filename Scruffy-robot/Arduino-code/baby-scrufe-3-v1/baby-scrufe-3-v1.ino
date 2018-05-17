#include <Servo.h>          // Incluir la librería Servo

Servo turret ;              // servo de la torreta, giro lateral de 40 a 128 grados
Servo up_sensor ;           // servo ultrasonido, giro atrás 7º, giro adelante 179º
Servo left_wheel ;          // servo rueda izquierda
Servo right_wheel ;         // servo rueda derecha

// sentido de la marcha y giros
#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define TURN_LEFT 3
#define TURN_RIGHT 4
#define LEFT_BACKWARD 5
#define RIGHT_BACKWARD 6

// velocidad de avance o retroceso
#define ZERO 0
#define VERYSLOW 3
#define SLOW 4          // 5
#define MEDIUM 5        // 10
#define FAST 6          // 25
#define ULTRAFAST 7     // 180

#define REDUCE_DISTANCE 50
#define SAFETY_DISTANCE 20

// depuración global
const boolean debuger = true ;

// pines del sensor de ultrasonido
const int EchoPin = 2 ;             // HC-SR04 pin del eco
const int TriggerPin = 7 ;          // HC-SR04 pin del disparador

// pin del infrarrojo anterior
const int infrared_front_Pin = 8 ;
// pin del infrarrojo posterior
const int infrared_rear_Pin = 2 ;

const int claxon = 12 ;              // claxon = buzzer

const int LED = 13 ;                // pin para el led de aviso ( las largas )

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

const int back_times = 2 ;
const int turn_times = 5 ;

// valores máximos y mínimos para el servo de turno
int min_turret_angle_value =  35 ;      // giro a la izquierda de la torreta
int max_turret_angle_value = 145 ;      // giro a la derecha de la torreta

int min_up_sensor_angle_value =  35 ;   // ángulo máximo sensor ultrasonido hacia atrás
int max_up_sensor_angle_value = 140 ;   // ángulo máximo sensor ultrasonido hacia adelante

// valor de lectura del infrarrojo anterior
int IR_front_value = 0 ;
// valor de lectura del infrarrojo posterior
int IR_rear_value = 0 ;

// pin digital conectado a la salida del sensor PIR (sin uso actual)
int Pin_PIR = 11 ;

// control de velocidad
int current_velocity = 0 ;              // velocidad actual ( SLOW, MEDIUM, FAST )
// sentido de la marcha actual
int current_direction = 0 ;             // sentido de giro actual (0 = STOP)
// contador de ciclos de movimiento
int cycle_count = 0 ;                   // evita que los cambios de marcha se sucedan rapidamente
// retardo general
int general_delay = 150 ;
// retardo general para giros de torreta y sensor
int sensor_delay = 900 ;
// retardo posicionamiento servos ruedas delantera
int servo_read_delay = 150 ;


int cm_derecha = 0 ;      // distancia disponible a la derecha
int cm_izquierda = 0 ;    // distancia disponible a la izquierda
int cm_abajo = 0 ;        // distancia disponible hacia abajo
int cm_arriba = 0 ;       // distancia disponible hacia arriba

boolean stop_moving = false ;     // indica error de lectura de ambos infrarojos


void setup() {
  if ( debuger ) {
    // conectar la salida serie para leer el valor del joystick
    Serial.begin(9600);
  }
      
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

  // dos segundos de espera antes de arrancar, calentando...
  delay( 2000 ) ;
 
}

void loop() {

comienzo:

  //
  // antes de comenzar a rodar, evaluar el escenario actual, puede que no sea posible  
  //
  
  // estatus del lector de infrarojos anterior y posterior
  IR_front_value = IR_front_status() ;
  IR_rear_value = IR_rear_status() ;
  
  if ( IR_front_value == HIGH ) {
    // no hay suelo, parada de emergencia
    if ( debuger ) {
      Serial.println( "2. ¡No detectado suelo delante! " ) ;
    }
    
    move_on( STOP, ZERO ) ;
    delay( sensor_delay ) ;

    // estimar la profundidad, si es asequible ( <= 8 cm ) seguir avanzando, continuar con normalidad
    cm_abajo = look_down() ;
    if ( cm_abajo <= 8 ) {
      move_on( FORWARD, VERYSLOW ) ;
    } else {

      if ( debuger ) {
        Serial.println( "3. Marcha atrás lenta " ) ;
      }

      // IMPORTANTE: controlar para que el bucle no se convierta en eterno
      while ( IR_front_status() == HIGH ) {
        // no hay suelo, detrás, parada si vamos marcha atrás
        if ( debuger ) {
          Serial.println( "4. ¡No detectado suelo detrás! " ) ;
        }
          
        // bucle hasta detectar suelo de nuevo, marcha atrás
        move_on( BACKWARD, VERYSLOW ) ;
        move_on( STOP, ZERO ) ;
        // delay( sensor_delay ) ;
    
        // si fallan ambos sensores: estamos volando o nos han cogido con la mano ¡cabrones!
        if ( IR_rear_status() == HIGH ) {
          if ( debuger ) {
            Serial.println( "5. ¡NO HAY SUELO NI DELANTE NI DETRÁS! stop_moving = true " ) ;
          }
          move_on( STOP, ZERO ) ;          
          // no hay suelo accesible, evitar movimientos mientras los infrarojos no den una lectura positiva
          stop_moving = true ;
          goto comienzo ;
        }
      }
      
      // hacia atrás para tener espacio de giro
      for ( int contador = 0 ; contador < back_times ; contador++ ) {
        move_on( BACKWARD, VERYSLOW ) ;      
        move_on( STOP, ZERO ) ;
      }
      delay( sensor_delay ) ;

      cm_izquierda = look_at_left() ;     // espacio izquierdo
      cm_derecha = look_at_right() ;      // espacio derecho

      if ( debuger) {
        Serial.println( "6. Izquierda: " + String( cm_izquierda ) + " Derecha: " + String( cm_derecha ) ) ;
      }

      if ( cm_derecha > cm_izquierda ) {
        for ( int contador = 0 ; contador < turn_times ; contador++ ) {
          move_on( TURN_RIGHT, VERYSLOW ) ;
        }
      } else if ( cm_izquierda > cm_derecha) {
        for ( int contador = 0 ; contador < turn_times ; contador++ ) {
          move_on( TURN_LEFT, VERYSLOW ) ;
        }
      } else {
        // podemos estar en una mesa o delante de un escalón
        for ( int contador = 0 ; contador < turn_times ; contador++ ) {
          move_on( TURN_RIGHT, VERYSLOW ) ;
        }
      }

      move_on( STOP, ZERO ) ;

      // después de retroceder volver a registrar la distancia del sensor principal
      int cm = look_at_front() ;
      
    }
  }

  // lectura del sensor de ultrasonido delantero, avance hacia adelante
  int cm = ping( TriggerPin, EchoPin ) ;
  
  if (debuger ) {
    Serial.println( "1. Distancia frontal: " + String( cm ) + " suelo: " + String( IR_front_value ) ) ;
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
  if ( cm <= SAFETY_DISTANCE ) {
    move_on( STOP, ZERO ) ;
    delay( sensor_delay ) ;

    // el objeto puede ser bajo pero a la altura del sensor e impedir el avance, o bien el ángulo de cálculo nos da una
    // estimación mayor así que puede ser una pared, una persona o un mueble...
    cm_arriba = look_up() ;

    // no podemos estimar el ángulo de cálculo para conocer la altura del objeto
    if ( cm_arriba > cm ) {
      // pared: girar hacia el lado libre
    }

    cm_izquierda = look_at_left() ;       // espacio izquierdo
    cm_derecha = look_at_right() ;        // espacio derecho

    if ( debuger) {
      Serial.println( "7. Izquierda: " + String( cm_izquierda ) + " Derecha: " + String( cm_derecha ) ) ;
    }

    // hacia atrás
    for ( int contador = 0 ; contador < back_times ; contador++ ) {
      move_on( BACKWARD, VERYSLOW ) ;      
    }
    move_on( STOP, ZERO ) ;
    
    if ( cm_derecha > cm_izquierda ) {
      for ( int contador = 0 ; contador < turn_times ; contador++ ) {
        move_on( TURN_RIGHT, VERYSLOW ) ;
      }
    } else if ( cm_izquierda > cm_derecha) {
      for ( int contador = 0 ; contador < turn_times ; contador++ ) {
        move_on( TURN_LEFT, VERYSLOW ) ;
      }
    } else {
      goto comienzo ;
    }

    move_on( STOP, ZERO ) ;

    // después de retroceder volver a registrar la distancia del sensor principal
    int cm = look_at_front() ;

  }

  // iniciar la marcha si hay suelo y no hay objetos cerca AL FRENTE
  if ( current_velocity == 0 && cm > SAFETY_DISTANCE && IR_front_value == LOW ) {
    // arranque en marcha lenta siempre que estemos parados
    if ( current_direction != FORWARD ) {
      move_on( FORWARD, SLOW );
    }
  }
  
  //
  //    estamos en marcha ( adelante )
  //
  
  // escenario, sentido de la marcha y posibles obstáculos
  switch ( current_direction ) {
      case FORWARD:
        if ( debuger ) {
          Serial.println( " -- Case FORWARD - ciclos " + String( cycle_count) + " velocidad: " +  String( current_velocity) ) ;
        }

        // distancia segura hacia delante, seguridad alta, distancia por encima de 50 cm
        if ( cm >= REDUCE_DISTANCE ) {
          if ( debuger ) {
            Serial.println( " acelerando --> ") ;
          }
          // incrementar SLOW --> MEDIUM
          if ( current_velocity == SLOW && cycle_count > 5 ) {
            move_on( FORWARD, MEDIUM ) ;
            cycle_count = -1 ;
          }

          // incrementar MEDIUM --> FAST
          if ( current_velocity == MEDIUM && cycle_count > 5 ) {
            move_on( FORWARD, FAST ) ;
            cycle_count = -1 ;
          }

          // incrementar FAST --> ULTRAFAST
          if ( current_velocity == FAST && cycle_count > 5 ) {
            move_on( FORWARD, ULTRAFAST ) ;
            cycle_count = -1 ;
          }

          // si vamos a tope, poner el contador de ciclos a cero para evitar desbordamiento de la variable
          if ( current_velocity == ULTRAFAST && cycle_count > 5 ) {
            cycle_count = -1 ;
          }
        } else {
          if ( debuger ) {
            Serial.println( " reduciendo --> ") ;
          }
          
          // reducir velocidad cuando estamos por debajo de la distancia de segurida mayor > 15 cm < 75 cm
          if ( current_velocity >= ULTRAFAST && cycle_count > 5 ) {
            move_on( STOP, ZERO ) ;
            move_on( FORWARD, SLOW ) ;
            cycle_count = -1 ;
          }

          if ( current_velocity == SLOW && cycle_count > 5 ) {
            cycle_count = -1 ;
          }
        }

      break;

      case BACKWARD:
        if ( debuger ) {
          Serial.println( " -- Case BACKWARD - ciclos " + String( cycle_count) ) ;
        }

        acoustic_claxon() ;
        
        if ( cycle_count > 3 ) {
          move_on( STOP, ZERO ) ;
          cycle_count = -1 ;
        }
      break;

      case TURN_LEFT:
        if ( debuger ) {
          Serial.println( " -- Case TURN_LEFT - ciclos " + String( cycle_count) ) ;
        }

        if ( cycle_count > 2 ) {
          move_on( STOP, ZERO ) ;
          cycle_count = -1 ;
        }
      break;
      
      case TURN_RIGHT:
        if ( debuger ) {
          Serial.println( " -- Case TURN_RIGHT - ciclos " + String( cycle_count) ) ;
        }

        if ( cycle_count > 2 ) {
          move_on( STOP, ZERO ) ;
          cycle_count = -1 ;
        }
      break;
      
      case LEFT_BACKWARD:
        if ( debuger ) {
          Serial.println( " -- Case LEFT_BACKWARD - ciclos " + String( cycle_count) ) ;
        }

        acoustic_claxon() ;
        
        if ( cycle_count > 2 ) {
          move_on( STOP, ZERO ) ;
          cycle_count = -1 ;
        }
      break;
      
      case RIGHT_BACKWARD:
        if ( debuger ) {
          Serial.println( " -- Case RIGHT_BACKWARD - ciclos " + String( cycle_count) ) ;
        }

        acoustic_claxon() ;

        if ( cycle_count > 2 ) {
          move_on( STOP, ZERO ) ;
          cycle_count = -1 ;
        }
      break;

  }

  cycle_count++ ;
  
  if ( debuger ) {
    Serial.println( "8. Dirección: " + String( current_direction ) + " velocidad " + String( current_velocity ) ) ;
    // Serial.print ( " - Turret: " + String( turret_angle ) + "º Sensor: " + String( up_sensor_angle ) );
    // Serial.print ( " LeftW: " + String( lw_angle ) + "º RightW: " + String( rw_angle ) );
    // Serial.print ( " - objeto a " + String(cm) + " cm " );
    // Serial.println();  
  }

  delay( general_delay ) ;
  
}

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
  if ( debuger ) {
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
    if ( debuger ) {
      Serial.println( "Excedido rango inferior" );
    }
    
    return 0 ;
  }
  
  if ( distanceCm > 300 ) { 
    if ( debuger ) {
      Serial.println( "Excededo rango superior" );
    }

    return 999 ;
  }
 
  return distanceCm;
  
}

//
//    lectura estado infrarrojo anterior
//
int IR_front_status() {
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
}

//
//    ordenes de movimiento completas
//
void move_on( int move_to, int velocity ) {

  // mover hacia adelante, controlar el infrarojo delantero
  if ( move_to == FORWARD || move_to == TURN_LEFT || move_to == TURN_RIGHT ) {
    int ir_front = IR_front_status() ;
    if ( ir_front == HIGH ) {
      current_velocity = ZERO ;
      current_direction = STOP ;
      delay( sensor_delay ) ;
      return ;
    }
  }

  // movimiento hacia atrás, controlar el infrarojo trasero
  if ( move_to == BACKWARD || move_to == LEFT_BACKWARD || move_to == RIGHT_BACKWARD ) {
    int ir_rear = IR_rear_status() ;
    if ( ir_rear == HIGH ) {
      current_velocity = ZERO ;
      current_direction = STOP ;
      delay( sensor_delay ) ;
      return ;
    }
  }
  
  // lectura posición actual de ambos servos
  lw_angle = left_wheel.read();
  rw_angle = right_wheel.read();

  if ( debuger ) {
    Serial.print ( " LeftW: " + String( lw_angle ) + "º RightW: " + String( rw_angle ) );
  }

  switch ( move_to ) {
    case STOP:
      left_wheel.write( initial_angle[ 2 ] ) ;
      right_wheel.write( initial_angle[ 3 ] ) ;

      if ( debuger ) { Serial.println( " Parado " ) ; }
    break;
    
    case FORWARD:
      left_wheel.write( lw_angle + velocity ) ;     // avanza
      right_wheel.write( rw_angle - velocity ) ;    // avanza
      look_at_front() ;
      
      if ( debuger ) { Serial.println( " Adelante " ) ; }
    break;
    
    case BACKWARD:
      left_wheel.write( lw_angle - velocity ) ;     // retrocede
      right_wheel.write( rw_angle + velocity ) ;    // retrocede
      look_at_front() ;

      acoustic_claxon() ;
      
      if ( debuger ) { Serial.println( " Atrás " ) ; }
    break;
    
    case TURN_LEFT:
      left_wheel.write( lw_angle - velocity ) ;     // retrocede
      right_wheel.write( rw_angle - velocity ) ;    // avanza

      if ( debuger ) { Serial.println( " Giro izquierda " ) ; }
    break;
    
    case TURN_RIGHT:
      left_wheel.write( lw_angle + velocity ) ;     // avanza
      right_wheel.write( rw_angle + velocity ) ;    // retrocede

      if ( debuger ) { Serial.println( " Giro derecha " ) ; }
    break;

    case LEFT_BACKWARD:
      left_wheel.write( lw_angle - velocity ) ;     // retrocede
      right_wheel.write( rw_angle - velocity ) ;    // avanza

      acoustic_claxon() ;

      if ( debuger ) { Serial.println( " Atrás izquierda " ) ; }
    break;

    case RIGHT_BACKWARD:
      left_wheel.write( lw_angle + velocity ) ;     // avanza
      right_wheel.write( rw_angle + velocity ) ;    // retrocede

      acoustic_claxon() ;
      
      if ( debuger ) { Serial.println( " Atrás derecha " ) ; }
    break;

    // igual que emergency_stop()
    default:
      left_wheel.write( initial_angle[ 2 ] ) ;
      right_wheel.write( initial_angle[ 3 ] ) ;
      current_velocity = 0 ;
      current_direction = 0 ;
      cycle_count = 0 ;
    break;
  }

  // guardamos la velocidad actual
  current_velocity = velocity ;
  //  guardamos la dirección actual
  current_direction = move_to ;

  // esperar hasta que el motor tome la nueva posición
  delay( servo_read_delay ) ;
/*
  if ( debuger ) {
    Serial.println( " Move_To, contando ciclos... " ) ;
  }
*/
}

//
//  señales acústicas y luminosas
// 
void acoustic_claxon() {

  // señal acústica de marcha atrás
  tone( claxon, 500 ) ;
  delay( 200 ) ;
  noTone( claxon ) ;
  
}

