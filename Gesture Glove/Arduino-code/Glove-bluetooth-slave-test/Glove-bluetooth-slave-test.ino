// glove bluetooth slave receive test

#include <SoftwareSerial.h>   // Incluimos la librería  SoftwareSerial  

SoftwareSerial BT_slave( 8, 9 );      // Definimos los pines RX y TX del Arduino conectados al Bluetooth

#define DEBUGER true
#define ARRAY_BT_COMMANDS 25

String old_cmd = "" ;
String new_cmd = "" ;
char cmd_rcv  ;
byte index ;

String bt_commands[ ARRAY_BT_COMMANDS ] = { "S", "f", "F", "b", "B", "r", "R", "l", "L", "q", "Q", "z", "Z", "e", "E", "c", "C", "u", "U", "d", "D", "1", "2", "3", "4" };
// String bt_response[ ARRAY_BT_COMMANDS ] = { "->S", "->f", "->F", "->b", "->B", "->r", "->R", "->l", "->L", "->q", "->Q", "->z", "->Z", "->e", "->E", "->c", "->C", "->u", "->U", "->d", "->D", "->1", "->2", "->3", "->4" };
String bt_response[ ARRAY_BT_COMMANDS ] = { "Stopping", "forward", "Fast forward", "backward", "Fast Backward", "rigth", "Fast Right", "left", "Fast Left", "Fordward left", "Fast Forward Left", "Backward left", "Fast Backward left", "Forward right", "Fast Forward right", "Backward right", "Fast Backward right", "up", "Fast Up", "down", " Fast Down", "Command 1", "Command 2", "Command 3", "Command 4" };

void setup() {
  // put your setup code here, to run once:

  BT_slave.begin( 38400 ) ;

  delay( 100 ) ;

  Serial.begin( 9600 );

  while (!Serial) {
    ;
  }

  Serial.println(" Depurando bluetooth glove slave.") ;

}

void loop() {
  // put your main code here, to run repeatedly:

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
      }
    }

    // error de comando
    if ( !found ) {
      BT_slave.println( "ERROR" );   // send response to master
    }
    
  }

  delay( 100 ) ;
}
