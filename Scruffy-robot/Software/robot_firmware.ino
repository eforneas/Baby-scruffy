#include <Servo.h>

int drive_motor_rate = 4;
int step_motor_rate = 2;
int servo_motor_rate = 2;

int servo_delay = 1;

int bottom_init = 90;
int middle_init = 180;
int top_init = 90;

int bottom_current;
int middle_current;
int top_current;

int topServoMin = 0;
int topServoMax = 180;
int middleServoMin = 80;
int middleServoMax = 180;
int bottomServoMin = 60;
int bottomServoMax = 170;

Servo topServo;
Servo middleServo;
Servo bottomServo;

int topServoPin = 6;
int middleServoPin = 7;
int bottomServoPin = 8;

int rightDriveMotorPin1 = 2;
int rightDriveMotorPin2 = 3;
int leftDriveMotorPin1 = 4;
int leftDriveMotorPin2 = 5;

int motorPin1 = 9;
int motorPin2 = 10;
int motorPin3 = 11;
int motorPin4 = 12;

int incomingByte;

void setup() { 
  
  Serial.begin(9600);
  
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  
  stepper_stop();

  pinMode(rightDriveMotorPin1, OUTPUT);
  pinMode(rightDriveMotorPin2, OUTPUT);
  pinMode(leftDriveMotorPin1, OUTPUT);
  pinMode(leftDriveMotorPin2, OUTPUT);
  
  stop_robot();
  
  topServo.attach(topServoPin);
  middleServo.attach(middleServoPin);
  bottomServo.attach(bottomServoPin);
  
  topServo.write(top_init);
  middleServo.write(middle_init);
  bottomServo.write(bottom_init);
  
  top_current = top_init;
  middle_current = middle_init;
  bottom_current = bottom_init;  
  
} 
 
 
void loop() { 
  
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    
    switch (incomingByte) {
      case 'w':
        move_forward(drive_motor_rate, 10, 0);
        break;
      case 's':
        move_back(drive_motor_rate, 10, 0);
        break;
      case 'a':
        move_left(drive_motor_rate, 10, 0);
        break;
      case 'd':
        move_right(drive_motor_rate, 10, 0);
        break;
      case 'h':
        stepper_left (step_motor_rate, 5);
        break;
      case 'l':
        stepper_right (step_motor_rate, 5);
        break;
      case 'y':
        top_move (top_current + servo_motor_rate);
        break;
      case 'o':
        top_move (top_current - servo_motor_rate);
        break;
      case 'u':
        middle_move (middle_current - servo_motor_rate);
        break;
      case 'j':
        middle_move (middle_current + servo_motor_rate);
        break;
      case 'i':
        bottom_move (bottom_current - servo_motor_rate);
        break;
      case 'k':
        bottom_move (bottom_current + servo_motor_rate);
        break;
    }
    
    Serial.print("Command: ");
    Serial.println(incomingByte);
  }
  
  //servo test
  /* 
  for(int pos = 90; pos < 160; pos = pos + 10)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    bottom_move(pos);              // tell servo to go to position in variable 'pos' 
    delay(1000);                       // waits 15ms for the servo to reach the position 
  } 
  for(int pos = 160; pos>=90; pos = pos - 10)     // goes from 180 degrees to 0 degrees 
  {                                
    bottom_move(pos);              // tell servo to go to position in variable 'pos' 
    delay(1000);                       // waits 15ms for the servo to reach the position 
  }
  */
  
  //drive motor test
  /*
  move_forward(1000, 6, 50);
  move_backward(100, 1);
  move_left(100, 1);
  move_right(100, 1);
  */
  
  //stepper motor test
  /*
  stepper_forward (100, 5);
  */
}

void bottom_move (int target) {
  if (target > bottom_current) {
    while (target != bottom_current) {
      if (bottom_current >= bottomServoMax) {
        break;
      }
      bottom_current++;
      bottomServo.write(bottom_current);
      delay(servo_delay);
    }
  } else if (target < bottom_current) {
    while (target != bottom_current) {
      if (bottom_current <= bottomServoMin) {
        break;
      }
      bottom_current--;
      bottomServo.write(bottom_current);
      delay(servo_delay);
    }
  }
}

void middle_move (int target) {
  if (target > middle_current) {
    while (target != middle_current) {
      if (middle_current >= middleServoMax) {
        break;
      }
      middle_current++;
      middleServo.write(middle_current);
      delay(servo_delay);
    }
  } else if (target < middle_current) {
    while (target != middle_current) {
      if (middle_current <= middleServoMin) {
        break;
      }
      middle_current--;
      middleServo.write(middle_current);
      delay(servo_delay);
    }
  }
}

void top_move (int target) {
  if (target > top_current) {
    while (target != top_current) {
      if (top_current >= topServoMax) {
        break;
      }
      top_current++;
      topServo.write(top_current);
      delay(servo_delay);
    }
  } else if (target < top_current) {
    while (target != top_current) {
      if (top_current <= topServoMin) {
        break;
      }
      top_current--;
      topServo.write(top_current);
      delay(servo_delay);
    }
  }
} 

void move_forward (int steps, int onDelay, int stopDelay) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(rightDriveMotorPin1, HIGH);
    digitalWrite(rightDriveMotorPin2, LOW);
    digitalWrite(leftDriveMotorPin1, HIGH);
    digitalWrite(leftDriveMotorPin2, LOW);
    delay(onDelay);
    digitalWrite(rightDriveMotorPin1, LOW);
    digitalWrite(rightDriveMotorPin2, LOW);
    digitalWrite(leftDriveMotorPin1, LOW);
    digitalWrite(leftDriveMotorPin2, LOW);
    delay(stopDelay);
  }    
}

void move_back (int steps, int onDelay, int stopDelay) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(rightDriveMotorPin1, LOW);
    digitalWrite(rightDriveMotorPin2, HIGH);
    digitalWrite(leftDriveMotorPin1, LOW);
    digitalWrite(leftDriveMotorPin2, HIGH);
    delay(onDelay);
    digitalWrite(rightDriveMotorPin1, LOW);
    digitalWrite(rightDriveMotorPin2, LOW);
    digitalWrite(leftDriveMotorPin1, LOW);
    digitalWrite(leftDriveMotorPin2, LOW);
    delay(stopDelay);
  }    
}

void move_left (int steps, int onDelay, int stopDelay) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(rightDriveMotorPin1, HIGH);
    digitalWrite(rightDriveMotorPin2, LOW);
    digitalWrite(leftDriveMotorPin1, LOW);
    digitalWrite(leftDriveMotorPin2, LOW);
    delay(onDelay);
    digitalWrite(rightDriveMotorPin1, LOW);
    digitalWrite(rightDriveMotorPin2, LOW);
    digitalWrite(leftDriveMotorPin1, LOW);
    digitalWrite(leftDriveMotorPin2, LOW);
    delay(stopDelay);
  }    
}

void move_right (int steps, int onDelay, int stopDelay) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(rightDriveMotorPin1, LOW);
    digitalWrite(rightDriveMotorPin2, LOW);
    digitalWrite(leftDriveMotorPin1, HIGH);
    digitalWrite(leftDriveMotorPin2, LOW);
    delay(onDelay);
    digitalWrite(rightDriveMotorPin1, LOW);
    digitalWrite(rightDriveMotorPin2, LOW);
    digitalWrite(leftDriveMotorPin1, LOW);
    digitalWrite(leftDriveMotorPin2, LOW);
    delay(stopDelay);
  }    
}

void stop_robot() {
    digitalWrite(rightDriveMotorPin1, LOW);
    digitalWrite(rightDriveMotorPin2, LOW);
    digitalWrite(leftDriveMotorPin1, LOW);
    digitalWrite(leftDriveMotorPin2, LOW);  
}

void stepper_right (int steps, int motorDelay) {
  
  for (int i = 0; i < steps; i++) {
    
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, HIGH);
    digitalWrite(motorPin4, LOW);
    delay(motorDelay);
   
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, HIGH);
    digitalWrite(motorPin4, HIGH);
    delay(motorDelay);  
   
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, HIGH);
    delay(motorDelay);  
   
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, HIGH);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, HIGH);
    delay(motorDelay); 
   
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, HIGH);
    delay(motorDelay); 
   
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    digitalWrite(motorPin3, HIGH);
    digitalWrite(motorPin4, HIGH);
    delay(motorDelay); 
   
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    digitalWrite(motorPin3, HIGH);
    digitalWrite(motorPin4, LOW);
    delay(motorDelay); 
   
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, HIGH);
    digitalWrite(motorPin3, HIGH);
    digitalWrite(motorPin4, LOW);
    delay(motorDelay); 
  }
}

void stepper_left (int steps, int motorDelay) {
 
  for (int i = 0; i < steps; i++) {
  
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, HIGH);
    digitalWrite(motorPin3, HIGH);
    digitalWrite(motorPin4, LOW);
    delay(motorDelay); 
   
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    digitalWrite(motorPin3, HIGH);
    digitalWrite(motorPin4, LOW);
    delay(motorDelay);  
   
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    digitalWrite(motorPin3, HIGH);
    digitalWrite(motorPin4, HIGH);
    delay(motorDelay);  
   
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, HIGH);
    delay(motorDelay); 
   
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, HIGH);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, HIGH);
    delay(motorDelay); 
   
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, HIGH);
    delay(motorDelay); 
   
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, HIGH);
    digitalWrite(motorPin4, HIGH);
    delay(motorDelay); 
   
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, HIGH);
    digitalWrite(motorPin4, LOW);
    delay(motorDelay); 
  }
 
}
 
void stepper_stop() {
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, HIGH);
}


