#define sensor0Pin 13 //middle sensor
#define sensor1Pin 12 //right first sensor
#define sensor2Pin 8 //right second sensor
#define sensor_1Pin 7 //left first sensor
#define sensor_2Pin 4 //left second sensor

#define ENA 11 //PWM signal for Left Motor
#define ENB 10 //PWM signal for Right Motor
#define IN1 9 //Direction control for Left (forward)
#define IN2 6 //Direction control for Left (backward)
#define IN3 5 //Direction control for Right (forward)
#define IN4 3 //Direction control for Right (backward)

int error = 0;
int previousError = 0;
int P = 0;
int I = 0;
int D = 0;
int Kp = 0;
int Ki = 0;
int Kd = 0;
int PIDvalue = 0;
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

void setup(void) {
  pinMode(sensor0Pin, INPUT);
  pinMode(sensor1Pin, INPUT);
  pinMode(sensor2Pin, INPUT);
  pinMode(sensor_1Pin, INPUT);
  pinMode(sensor_2Pin, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() {
  int LFSensor[5]={0, 0, 0, 0, 0};
  LFSensor[0] = digitalRead(sensor_2Pin);
  LFSensor[1] = digitalRead(sensor_1Pin);
  LFSensor[2] = digitalRead(sensor0Pin);
  LFSensor[3] = digitalRead(sensor1Pin);
  LFSensor[4] = digitalRead(sensor2Pin);

  if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 )) {
    error = 4;
    P = error;
    I = I + error;
    D = error - previousError;
    PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
    previousError = error;

    leftMotorSpeed = 128 + PIDvalue;
    rightMotorSpeed = 128 - PIDvalue;

    analogWrite(ENA, leftMotorSpeed);
    analogWrite(ENB, rightMotorSpeed);
  }

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )) {
    error = 3;
    P = error;
    I = I + error;
    D = error - previousError;
    PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
    previousError = error;

    leftMotorSpeed = 128 + PIDvalue;
    rightMotorSpeed = 128 - PIDvalue;

    analogWrite(ENA, leftMotorSpeed);
    analogWrite(ENB, rightMotorSpeed);
  }

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )) {
    error = 2;
    P = error;
    I = I + error;
    D = error - previousError;
    PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
    previousError = error;

    leftMotorSpeed = 128 + PIDvalue;
    rightMotorSpeed = 128 - PIDvalue;

    analogWrite(ENA, leftMotorSpeed);
    analogWrite(ENB, rightMotorSpeed);
  }

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )) {
    error = 1;
    P = error;
    I = I + error;
    D = error - previousError;
    PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
    previousError = error;

    leftMotorSpeed = 128 + PIDvalue;
    rightMotorSpeed = 128 - PIDvalue;

    analogWrite(ENA, leftMotorSpeed);
    analogWrite(ENB, rightMotorSpeed);
  }

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) {
    error = 0;
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
    previousError = 0;
  }

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) {
    error = -1;
    P = error;
    I = I + error;
    D = error - previousError;
    PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
    previousError = error;

    leftMotorSpeed = 128 - PIDvalue;
    rightMotorSpeed = 128 + PIDvalue;

    analogWrite(ENA, leftMotorSpeed);
    analogWrite(ENB, rightMotorSpeed);
  }

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) {
    error = -2;
    P = error;
    I = I + error;
    D = error - previousError;
    PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
    previousError = error;

    leftMotorSpeed = 128 - PIDvalue;
    rightMotorSpeed = 128 + PIDvalue;

    analogWrite(ENA, leftMotorSpeed);
    analogWrite(ENB, rightMotorSpeed);
  }

  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) {
    error = -3;
    P = error;
    I = I + error;
    D = error - previousError;
    PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
    previousError = error;

    leftMotorSpeed = 128 - PIDvalue;
    rightMotorSpeed = 128 + PIDvalue;

    analogWrite(ENA, leftMotorSpeed);
    analogWrite(ENB, rightMotorSpeed);  
  }

  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) {
    error = -4;
    P = error;
    I = I + error;
    D = error - previousError;
    PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
    previousError = error;

    leftMotorSpeed = 128 - PIDvalue;
    rightMotorSpeed = 128 + PIDvalue;

    analogWrite(ENA, leftMotorSpeed);
    analogWrite(ENB, rightMotorSpeed);
  }
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )) {
    error = 0;
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
    previousError = 0;
  }
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) {
    error = 0;
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
    previousError = 0;
  }
}
