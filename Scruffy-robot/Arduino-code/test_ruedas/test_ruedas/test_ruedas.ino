#include <Servo.h>          // Incluir la librería Servo

Servo servo1;               // Crear un objeto tipo Servo llamado servo1
Servo servo2;               // Crear un objeto tipo Servo llamado servo2
Servo servo3;               // Crear un objeto tipo Servo llamado servo2
Servo servo4;               // Crear un objeto tipo Servo llamado servo2
Servo srv_test;             // servo fantasma de prueba

boolean debuger = true;

static int servo_par  = 1 ;        // pareja de servos conmutados por el botón del joystick

int angulo_s1 = 90 ;              // ángulo inicial del servo 1
int angulo_s2 = 90 ;              // ángulo inicial del servo 2
int angulo_s3 = 90 ;              // ángulo inicial del servo 3
int angulo_s4 = 90 ;              // ángulo inicial del servo 4

int angulo_p1 = 90 ;              // ángulo inicial del servo pinza 1
int angulo_p2 = 90 ;              // ángulo inicial del servo pinza 2

int Eje_X = A1 ;
int Eje_Y = A2 ;

int boton = 2  , LED = 13 ;

int lastSwitchState = HIGH;
boolean mouseIsActive = false;    // whether or not to control the mouse

int salto_s1 = 1 ;                 // Controla el salto por movimiento
int salto_s2 = 1 ;                 // Controla el salto por movimiento

int pin_servo1 = 9;
int pin_servo2 = 10;
int pin_servo3 = 11;
int pin_servo4 = 6;

// valores máximos y mínimos para el servo de turno
int min_servo1_angle_value = 1;    // SM-S2309S = 0  // Futaba S3003 = 4
int max_servo1_angle_value = 179;  // SM-S2309S = 178 // Futaba S3003 = 174

int min_servo2_angle_value = 1;   // SM-S2309S = 0  // Futaba S3003 = 4
int max_servo2_angle_value = 128;  // SM-S2309S = 178 // Futaba S3003 = 174

int min_servo3_angle_value = 1;    // SM-S2309S = 0  // Futaba S3003 = 4
int max_servo3_angle_value = 179;   // SM-S2309S = 178 // Futaba S3003 = 174

int min_servo4_angle_value = 1;    // SM-S2309S = 0  // Futaba S3003 = 4
int max_servo4_angle_value = 179;  // SM-S2309S = 178 // Futaba S3003 = 174

int servo_read_delay = 70;        // SM-S2309S = 80 // Futaba S3003 = 120

void setup() {
    if ( debuger ) {
      // conectar la salida serie para leer el valor del joystick
      Serial.begin(9600);
    }
      
    servo1.attach( pin_servo1 ) ;   // Conectar servo1 al pin definido en pin_servo1
    servo2.attach( pin_servo2 ) ;   // Conectar servo1 al pin definido en pin_servo2
    servo3.attach( pin_servo3 ) ;   // Conectar servo1 al pin definido en pin_servo2
    servo4.attach( pin_servo4 ) ;   // Conectar servo1 al pin definido en pin_servo2
    
    pinMode( boton, INPUT_PULLUP) ; // el switch del joystick va conectado al pin 2
    pinMode( LED, OUTPUT);          // pin del LED como output

    // servo1 al valor inicial que indique la variable
    servo1.write(angulo_s1);
    servo2.write(angulo_s2);
    servo3.write(angulo_s3);
    servo4.write(angulo_s4);

    // lectura de la posición inicial de arranque, par de servos 1 y 2
    angulo_s1 = servo1.read();
    angulo_s2 = servo2.read();
    angulo_s3 = servo3.read();
    angulo_s4 = servo4.read();

    // digitalWrite( LED, LOW);

    // servo_par = 1;
}

void loop() {
    if ( debuger ) {
      // Serial.println( "Valor joystick: " + String( analogRead(A1) ) + " servo1: " + String( angulo_s1 ) + "º servo2: " + String( angulo_s2 ) );
      Serial.println( "Joystick: " + String( analogRead(A1) ) + " S1: " + String( angulo_s1 ) + "º S2: " + String( angulo_s2 ) );
      Serial.println( "Joystick: " + String( analogRead(A2) ) + " S3: " + String( angulo_s3 ) + "º S4: " + String( angulo_s4 ) );
      Serial.println( "Botón :"  + String( boton ) );
      // mejora la lectura con el retraso
      //delay( 250 );
    }
    
    int valorjoy_X = analogRead( A1 );
    int valorjoy_Y = analogRead( A2 );
    
    if ( !mouseIsActive ) {
      if ( valorjoy_X < 400 )                   // Si la lectura es menor de 400
            angulo_s1 = angulo_s1 - salto_s1 ;  // disminuimos el angulo
      else if ( valorjoy_X > 600)               // Si mayor de 600
            angulo_s1 = angulo_s1 + salto_s1 ;  // Aumentamos el angulo
          
      if ( valorjoy_Y < 400 )                   // Si la lectura es menor de 400
            angulo_s2 = angulo_s2 - salto_s2 ;  // disminuimos el angulo
      else if ( valorjoy_Y > 600)               // Si mayor de 600
            angulo_s2 = angulo_s2 + salto_s2 ;  // Aumentamos el angulo
      
      // controlar que la variable tenga un valor comprendido entre el ángulo mínimo y el máximo
      if ( angulo_s1 < min_servo1_angle_value ) angulo_s1 = min_servo1_angle_value ;
      if ( angulo_s1 > max_servo1_angle_value ) angulo_s1 = max_servo1_angle_value;
  
      // controlar que la variable tenga un valor comprendido entre el ángulo mínimo y el máximo
      if ( angulo_s2 < min_servo2_angle_value ) angulo_s2 = min_servo2_angle_value ;
      if ( angulo_s2 > max_servo2_angle_value ) angulo_s2 = max_servo2_angle_value;
  
      // lectura del ángulo actual, mover solo si distinto
      if ( servo1.read() != angulo_s1 ) {
          servo1.write(angulo_s1);                     // Y este es el que mueve el servo
      }
  
      // lectura del ángulo actual, mover solo si distinto
      if ( servo2.read() != angulo_s2 ) {
          servo2.write(angulo_s2);                     // Y este es el que mueve el servo
      }
    } else {
      if ( valorjoy_X < 400 )                   // Si la lectura es menor de 400
            angulo_s3 = angulo_s3 - salto_s1 ;  // disminuimos el angulo
      else if ( valorjoy_X > 600)               // Si mayor de 600
            angulo_s3 = angulo_s3 + salto_s1 ;  // Aumentamos el angulo
          
      if ( valorjoy_Y < 400 )                   // Si la lectura es menor de 400
            angulo_s4 = angulo_s4 - salto_s2 ;  // disminuimos el angulo
      else if ( valorjoy_Y > 600)               // Si mayor de 600
            angulo_s4 = angulo_s4 + salto_s2 ;  // Aumentamos el angulo
      
      // controlar que la variable tenga un valor comprendido entre el ángulo mínimo y el máximo
      if ( angulo_s3 < min_servo3_angle_value ) angulo_s3 = min_servo3_angle_value ;
      if ( angulo_s3 > max_servo3_angle_value ) angulo_s3 = max_servo3_angle_value;
  
      // controlar que la variable tenga un valor comprendido entre el ángulo mínimo y el máximo
      if ( angulo_s4 < min_servo4_angle_value ) angulo_s4 = min_servo4_angle_value ;
      if ( angulo_s4 > max_servo4_angle_value ) angulo_s4 = max_servo4_angle_value;
  
      // lectura del ángulo actual, mover solo si distinto
      if ( servo3.read() != angulo_s3 ) {
          servo3.write(angulo_s3);                     // Y este es el que mueve el servo
      }
  
      // lectura del ángulo actual, mover solo si distinto
      if ( servo4.read() != angulo_s4 ) {
          servo4.write(angulo_s4);                     // Y este es el que mueve el servo
      }
    }
    
    delay( servo_read_delay );                              // Este delay regula la velocidad del movimiento
    
    // lectura del botón en el PIN 2
    int switchState = digitalRead(boton);
    if (switchState != lastSwitchState) {
      if (switchState == HIGH) {
        mouseIsActive = !mouseIsActive;
        // cambiar el LED para indicar el estado del botón
        digitalWrite(LED, mouseIsActive);

        // resetear ángulos
        if  ( mouseIsActive ) {
            angulo_s1 = 90;
            angulo_s2 = 90;
            angulo_s3 = 90;
            angulo_s4 = 90;

            servo1.write(angulo_s1) ;
            servo2.write(angulo_s2) ;
            servo3.write(angulo_s3) ;
            servo4.write(angulo_s4) ;

            // no guardamos el estado del botón, solo para reiniciar la posición de los servos
            mouseIsActive = false ;
        }
         

//        if ( mouseIsActive ) {
//          servo_par = 2; 
//
//          angulo_s3 = servo3.read();
//          angulo_s4 = servo4.read();
//          
//        } else {
//          servo_par = 1;
//
//          angulo_s1 = servo1.read();
//          angulo_s2 = servo2.read();
//          
//        }
      }
    }
    // salvar el estado del switch
    lastSwitchState = switchState;
    

/*    
    if ( ! digitalRead(boton) ) {
          digitalWrite(LED, HIGH);                      // encender LED vía PIN 13
          // angulo_s1 = angulo_s2 = 90;
          // conmuta la pareja de servos a manejar
          if ( servo_par = 1 ) {
              servo_par = 2;
          } else {
            if ( servo_par = 2 ) {
                servo_par = 1;
            }
          }

          delay( 250 );
    }
    else
        digitalWrite(LED, LOW);                         // LED apago switch
    
    // delay(250) ;
*/
        
}

// función que lleva un servo a un ángulo inicial de posición de forma suave
void servo1_start() {

  int servo_current = 0;

}

