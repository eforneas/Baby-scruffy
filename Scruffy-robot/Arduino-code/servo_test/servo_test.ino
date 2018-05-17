#include <Servo.h>          // Incluir la librería Servo

Servo turret ;              // servo de la torreta, giro lateral de 40 a 128 grados
Servo up_sensor ;           // servo ultrasonido, giro atrás 7º, giro adelante 179º
Servo left_wheel ;          // servo rueda izquierda
Servo right_wheel ;         // servo rueda derecha

// depuración global
const boolean debuger = false ;

// pines del sensor de ultrasonido
const int EchoPin = 4 ;
const int TriggerPin = 6 ;

// pin del infrarrojo
const int infraredPin = 8 ;

int turret_angle = 85 ;             // ángulo inicial torreta
int up_sensor_angle = 90 ;          // ángulo inicial del sensor de ultrasonido
int lw_angle = 90 ;                 // ángulo inicial rueda izquierda
int rw_angle = 90 ;                 // ángulo inicial rueda derecha

int Eje_X = A1 ;
int Eje_Y = A2 ;

// botón del joystick para pruebas
const int boton = 2 ;

// estado del pulsador
int lastSwitchState = HIGH ;
boolean mouseIsActive = false ;    // whether or not to control the mouse

const int LED = 13 ;

int salto_s1 = 2 ;                 // Controla el ángulo de cada salto por movimiento
int salto_s2 = 3 ;                 // Controla el ángulo de cada salto por movimiento

int rotacion_1 = 180 ;
int rotacion_2 = 180 ;

const int Pin_turret = 10 ;
const int Pin_up_sensor = 9 ;
const int Pin_left_wheel = 5 ;
const int Pin_right_wheel = 11 ;

// valores máximos y mínimos para el servo de turno
int min_turret_angle_value = 40 ;
int max_turret_angle_value = 128 ;

int min_up_sensor_angle_value = 25 ;
int max_up_sensor_angle_value = 165 ;

int servo_read_delay = 200 ;

// valor de lectura del infrarrojo
int IRvalue = 0 ;

int Pin_PIR = 11 ;      //pin digital conectado a la salida del sensor PIR

// control de velocidad
int current_velocity = 0 ;

void setup() {
  if ( debuger ) {
    // conectar la salida serie para leer el valor del joystick
    Serial.begin(9600);
  }
      
  turret.attach( Pin_turret ) ;             // Conectar turret al pin definido en pin_turret
  up_sensor.attach( Pin_up_sensor ) ;       // Conectar up_sensor al pin definido en pin_up_sensor
  left_wheel.attach( Pin_left_wheel ) ;     // Conectar left_wheel al pin definido en Pin_left_wheel
  right_wheel.attach( Pin_right_wheel ) ;   // Conectar right_wheel al pin definido en Pin_right_wheel
  
  pinMode( boton, INPUT_PULLUP ) ;  // el switch del joystick va conectado al pin 2
  
  pinMode( LED, OUTPUT ) ;          // pin del LED como output
  
  pinMode( infraredPin, INPUT ) ;   // Pin del infrarojo
  
  pinMode( TriggerPin, OUTPUT );    // Pin ultrasonido Trigger
  pinMode( EchoPin, INPUT );        // Pin ultrasonido Eco entrante
  
  pinMode( Pin_PIR, INPUT );

  digitalWrite( EchoPin, LOW );
  digitalWrite( TriggerPin, LOW );

  // servos al valor inicial que indique la variable de posición inicial
  turret.write( turret_angle ) ;
  up_sensor.write( up_sensor_angle ) ;
  left_wheel.write( lw_angle ) ;
  right_wheel.write( rw_angle ) ;

  // lectura de la posición inicial de arranque de todos los servos una vez posicionados
  turret_angle = turret.read();
  up_sensor_angle = up_sensor.read();
  
  // ruedas paradas
  lw_angle = left_wheel.read();
  rw_angle = right_wheel.read();

  // apagar el LED
  digitalWrite( LED, LOW );
 
}

void loop() {

  // lectura del infrarrojo anterior
  IRvalue = IR_status() ;

  if ( IRvalue == HIGH ) {
      // no hay suelo, parada de emergencia
      if ( debuger ) {
        Serial.print( " ¡No se detectó suelo! " ) ;
      }
      
      emergency_stop();

      
  }

  // lectura del sensor de ultrasonido
  
  // int cm = ping( TriggerPin, EchoPin );
/*  
  // parar si hay un objeto cerca
  if ( cm <= 20 ) {
      emergency_stop() ;
  }
*/  
  // iniciar la marcha si hay suelo y no hay objetos cerca
  // if ( current_velocity == 0 ) {
    start_run_forward( 25 );
  // }
    
  if ( debuger ) {
    Serial.print ( " - Turret: " + String( turret_angle ) + "º Sensor: " + String( up_sensor_angle ) );
    Serial.print ( " LeftW: " + String( lw_angle ) + "º RightW: " + String( rw_angle ) );
    // Serial.print ( "Botón :"  + String( boton ) );
    // Serial.print ( " - objeto a " + String(cm) + " cm " );
    // mejora la lectura con el retraso
    // delay( 50 );
  }
    
    int valorjoy_X = analogRead( A1 );
    int valorjoy_Y = analogRead( A2 );

/*
    // para testear la torreta y el sensor    
    if ( valorjoy_X < 400 )                   // Si la lectura es menor de 400
          turret_angle = turret_angle - salto_s1 ;  // disminuimos el angulo
    else if ( valorjoy_X > 600)               // Si mayor de 600
          turret_angle = turret_angle + salto_s1 ;  // Aumentamos el angulo
        
    if ( valorjoy_Y < 400 )                   // Si la lectura es menor de 400
          up_sensor_angle = up_sensor_angle - salto_s2 ;  // disminuimos el angulo
    else if ( valorjoy_Y > 600)               // Si mayor de 600
          up_sensor_angle = up_sensor_angle + salto_s2 ;  // Aumentamos el angulo
    
    // controlar que la variable tenga un valor comprendido entre el ángulo mínimo y el máximo
    if ( turret_angle < min_turret_angle_value ) turret_angle = min_turret_angle_value ;
    if ( turret_angle > max_turret_angle_value ) turret_angle = max_turret_angle_value;

    // controlar que la variable tenga un valor comprendido entre el ángulo mínimo y el máximo
    if ( up_sensor_angle < min_up_sensor_angle_value ) up_sensor_angle = min_up_sensor_angle_value ;
    if ( up_sensor_angle > max_up_sensor_angle_value ) up_sensor_angle = max_up_sensor_angle_value;

    // lectura del ángulo actual, mover solo si distinto
    if ( turret.read() != turret_angle ) {
        turret.write( turret_angle );                     // Y este es el que mueve el servo
    }

    // lectura del ángulo actual, mover solo si distinto
    if ( up_sensor.read() != up_sensor_angle ) {
        up_sensor.write( up_sensor_angle );                     // Y este es el que mueve el servo
    }
*/    

    // para testear las ruedas    
    if ( valorjoy_X < 400 )                   // Si la lectura es menor de 400
          lw_angle = lw_angle - rotacion_1 ;  // disminuimos el angulo
    else if ( valorjoy_X > 600)               // Si mayor de 600
          lw_angle = lw_angle + rotacion_1 ;  // Aumentamos el angulo
        
    if ( valorjoy_Y < 400 )                   // Si la lectura es menor de 400
          rw_angle = rw_angle - rotacion_2 ;  // disminuimos el angulo
    else if ( valorjoy_Y > 600)               // Si mayor de 600
          rw_angle = rw_angle + rotacion_2 ;  // Aumentamos el angulo
/*    
    // controlar que la variable tenga un valor comprendido entre el ángulo mínimo y el máximo
    if ( turret_angle < min_turret_angle_value ) turret_angle = min_turret_angle_value ;
    if ( turret_angle > max_turret_angle_value ) turret_angle = max_turret_angle_value;

    // controlar que la variable tenga un valor comprendido entre el ángulo mínimo y el máximo
    if ( up_sensor_angle < min_up_sensor_angle_value ) up_sensor_angle = min_up_sensor_angle_value ;
    if ( up_sensor_angle > max_up_sensor_angle_value ) up_sensor_angle = max_up_sensor_angle_value;
*/
    // lectura del ángulo actual, mover solo si distinto
    if ( left_wheel.read() != lw_angle ) {
        left_wheel.write( lw_angle );                     // Y este es el que mueve el servo
    }

    // lectura del ángulo actual, mover solo si distinto
    if ( right_wheel.read() != rw_angle ) {
        right_wheel.write( rw_angle );                     // Y este es el que mueve el servo
    }

    // lectura del botón en el PIN 2
    int switchState = digitalRead( boton );
    if ( switchState != lastSwitchState ) {
      if (switchState == HIGH) {
        mouseIsActive = !mouseIsActive;
        // cambiar el LED para indicar el estado del botón
        digitalWrite( LED, mouseIsActive );

        // resetear ángulos
        if  ( mouseIsActive ) {
            turret_angle = 85 ;
            up_sensor_angle = 90 ;
            lw_angle = 90 ;
            rw_angle = 90 ;

            turret.write( turret_angle ) ;
            up_sensor.write( up_sensor_angle ) ;
            left_wheel.write( lw_angle ) ;
            right_wheel.write( rw_angle ) ;

            // no guardamos el estado del botón, solo para reiniciar la posición de los servos
            mouseIsActive = false ;
        }
      }
    }
    
    // salvar el estado del switch
    lastSwitchState = switchState;

  // salto línea si debuger activado
  if ( debuger ) {
    Serial.println();  
  }
        
}

// función que inicializa la posiciń de los servos
void servo_start() {

  int servo_current = 0;

}

int ping(int TriggerPin, int EchoPin) {
  
 long duration, distanceCm, distance;

repeat:

/*
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  // pinMode( TriggerPin, OUTPUT );
  digitalWrite( EchoPin, LOW );
  digitalWrite( TriggerPin, LOW );
  delayMicroseconds( 2 );
  digitalWrite( TriggerPin, HIGH );
  delayMicroseconds( 10 );
  digitalWrite( TriggerPin, LOW );

  // pinMode( EchoPin, INPUT );
  duration = pulseIn( EchoPin, HIGH, 20000 );
*/

/*
  digitalWrite(TriggerPin, LOW);  //para generar un pulso limpio ponemos a LOW 4us
  delayMicroseconds(10);
  digitalWrite(TriggerPin, HIGH);  //generamos Trigger (disparo) de 10us
  delayMicroseconds(10);
  digitalWrite(TriggerPin, LOW);
 
  duration = pulseIn(EchoPin, HIGH);  //medimos el tiempo entre pulsos, en microsegundos
*/

  digitalWrite( TriggerPin, LOW );
  digitalWrite( EchoPin, LOW );
  delayMicroseconds( 2 );
   
  digitalWrite( TriggerPin, HIGH ); // We send a 10us pulse
  delayMicroseconds( 10 );
  digitalWrite( TriggerPin, LOW );
 
  duration = pulseIn( EchoPin, HIGH ); // We wait for the echo to come back, with a timeout of 20ms, which corresponds approximately to 3m
  // pulseIn will only return 0 if it timed out. (or if echoPin was already to 1, but it should not happen)
  if(duration == 0) { // If we timed out
    pinMode( EchoPin, OUTPUT ); // Then we set echo pin to output mode
    digitalWrite( EchoPin, LOW ); // We send a LOW pulse to the echo pin
    delayMicroseconds( 200 );
    pinMode( EchoPin, INPUT ); // And finaly we come back to input mode
  }
 
distance = ( duration / 2 ) / 29.1 ; // We calculate the distance (sound speed in air is aprox. 291m/s), /2 because of the pulse going and coming
 
if (distance>3) { // Sensors are not capable for measuring small distances so i have avoided them 
  Serial.print("Distance: ");
  Serial.println(distance);
}
  if ( duration <= 200 ) {
    if ( debuger ) {
      Serial.println( " - Sensor tapado - " );
    }

    // parada de emergencia hasta volver a tener lectura del sensor
    emergency_stop();
    
    goto repeat;
  }

  // depuramos la duración del eco recogido
  if ( debuger ) {
    Serial.print( " - Duración: " + String( duration ) + " "  );
  }

  // distanceCm = duration * 10 / 292/ 2;   //convertimos a distancia, en cm
  distanceCm = int( 0.017 * duration );

  // distancia no válida, evita lecturas mayores de 200 cm, aunque parece que alcanza hasta 300 cm
  // también para valores por debajo de 0
 // if ( distanceCm > 200 || distanceCm < 0 ) { goto repeat; }

  if ( distanceCm <= 3 ) {
    if ( debuger ) {
      Serial.println( String( " - Error lectura: " + distanceCm) );
    }

    // parada de emergencia hasta volver a tener lectura del sensor
    emergency_stop();
    
    goto repeat;
  }

   if ( distanceCm > 300 ) { 
    if ( debuger ) {
      Serial.println( String( " - Error lectura: " + distanceCm) );    
    }

    // parada de emergencia hasta volver a tener lectura del sensor
    emergency_stop();
    
    goto repeat; 
  }
 
 // return distanceCm ;
 return distance ;
 
}

int IR_status() {
  int IR_front = digitalRead( infraredPin ) ;
  return IR_front ;
}

void emergency_stop() {
  left_wheel.write( 90 ) ;
  right_wheel.write( 90 ) ;
  
  current_velocity = 0 ;
}

void start_run_forward( int velocity ) {
  
  int ir_front = IR_status() ;

  if ( ir_front == HIGH ) {
    return ;
  }

  // poner ambas ruedas a la velocidad indicada hacia adelante
  lw_angle = left_wheel.read();
  rw_angle = right_wheel.read();
  
  left_wheel.write( lw_angle + velocity ) ;
  right_wheel.write( rw_angle - velocity ) ;

  // guardamos la velocidad actual
  current_velocity = velocity ;

  // esperar hasta que el motor tome la nueva posición
  delay( servo_read_delay ) ;
  
}

void start_run_backguard( int velocity ) {
  
  int ir_front = IR_status() ;

  if ( ir_front == HIGH ) {
    return ;
  }

  // poner ambas ruedas a la velocidad indicada hacia adelante
  lw_angle = left_wheel.read();
  rw_angle = right_wheel.read();
  
  left_wheel.write( lw_angle - velocity ) ;
  right_wheel.write( rw_angle + velocity ) ;

  // guardamos la velocidad actual
  current_velocity = velocity ;

  // esperar hasta que el motor tome la nueva posición
  delay( servo_read_delay ) ;

}


void left_wheel_forward( int velocity ) {

  int ir_front = IR_status() ;

  if ( ir_front == HIGH ) {
    return ;
  }

  // poner ambas ruedas a la velocidad indicada hacia adelante
  lw_angle = left_wheel.read();
  
  left_wheel.write( lw_angle + velocity ) ;

  // guardamos la velocidad actual
  current_velocity = velocity ;

  // esperar hasta que el motor tome la nueva posición
  delay( servo_read_delay ) ;

}

void right_wheel_forward( int velocity ) {

  int ir_front = IR_status() ;

  if ( ir_front == HIGH ) {
    return ;
  }

  // poner ambas ruedas a la velocidad indicada hacia adelante
  rw_angle = right_wheel.read();
  
  right_wheel.write( rw_angle - velocity ) ;

  // guardamos la velocidad actual
  current_velocity = velocity ;

  // esperar hasta que el motor tome la nueva posición
  delay( servo_read_delay ) ;

}

