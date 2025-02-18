
#define IN1 9
#define IN2 8
#define IN3 7
#define IN4 6
#define speedR 5
#define speedL 10
#define sensorL 4
#define sensorR 3
#define trigPin 11
#define echoPin 12
#define buzzer A1
#define LED_BLUETOOTH A2
#define LED_LINEFOLLOW 13
#define MODE_BUTTON 2
// motor initial speed and sensor readings
#define MOTOR_SPEED 120
int reading = 120;
int sl = 0;
int sr = 0;


// interrupt and button state variables
bool isBluetoothMode = false;

// function declaration
void left(int);
void right(int);
void stopp();
void forward(int);
void backward(int);
void handleBluetoothControl();
void handleLineFollowing();
void updateLEDs();
void ISR_btn();

void setup() {
  for (int i = 5; i <= 11; i++) {
    pinMode(i, OUTPUT);
  }
  pinMode(sensorR, INPUT);
  pinMode(sensorL, INPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(LED_BLUETOOTH, OUTPUT);
  pinMode(LED_LINEFOLLOW, OUTPUT);
  pinMode(MODE_BUTTON, INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println("Initializing...");


  // update LED status
  updateLEDs();
  rotateMotor(0, 0);

  //attach interrupt for button's mode
  // triggers the ISR_btn() whenever the state of MODE_BUTTON changes
  attachInterrupt(digitalPinToInterrupt(MODE_BUTTON), ISR_btn, RISING);
}

void loop() {

  // check mode then process the right mode
  if (isBluetoothMode) {
    handleBluetoothControl();
  } else {
    handleLineFollowing();
  }
  // update LED status
  updateLEDs();
  delay(500);
}

// interrupt service routine toggle between the two modes 
void ISR_btn() {

  isBluetoothMode = !isBluetoothMode;
  Serial.println(isBluetoothMode ? "Mode: Bluetooth" : "Mode: Line Follow");
}

// handle Bluetooth control commands
void handleBluetoothControl() {
  if (Serial.available()) {
    char command = Serial.read();
    Serial.println(command);
    if (command == 'F') forward(reading);
    else if (command == 'B') backward(reading);
    else if (command == 'L') left(reading);
    else if (command == 'R') right(reading);
    else if (command == 'S') stopp();
  }
}

// handle line following logic
void handleLineFollowing() {
  int rightIRSensorValue = digitalRead(sensorR);
  int leftIRSensorValue = digitalRead(sensorL);

  //If none of the sensors detects black line, then go straight
  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW) {
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
  }
  //If right sensor detects black line, then turn right
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW) {
    rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);
  }
  //If left sensor detects black line, then turn left
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH) {
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
  }
  //If both the sensors detect black line, then stop
  else {
    rotateMotor(0, 0);
  }

}


void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {

  if (rightMotorSpeed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else if (rightMotorSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  if (leftMotorSpeed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else if (leftMotorSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  analogWrite(speedR, abs(rightMotorSpeed));
  analogWrite(speedL, abs(leftMotorSpeed));
}


// motor control functions
void forward(int reading) {
  digitalWrite(IN1, HIGH);  //Left Motor backword Pin
  digitalWrite(IN2, LOW);   //Left Motor forword Pin
  digitalWrite(IN3, HIGH);  //Right Motor forword Pin
  digitalWrite(IN4, LOW);   //Right Motor backword Pin
  analogWrite(speedL, reading);
  analogWrite(speedR, reading);
}

void backward(int reading) {
  digitalWrite(IN1, LOW);   //Left Motor backword Pin
  digitalWrite(IN2, HIGH);  //Left Motor forword Pin
  digitalWrite(IN3, LOW);   //Right Motor forword Pin
  digitalWrite(IN4, HIGH);  //Right Motor backword Pin
  analogWrite(speedL, reading);
  analogWrite(speedR, reading);
}

void stopp() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(speedL, 0);
  analogWrite(speedR, 0);
}

void left(int reading) {
  digitalWrite(IN1, LOW);   //Left Motor backword Pin
  digitalWrite(IN2, HIGH);  //Left Motor forword Pin
  digitalWrite(IN3, HIGH);  //Right Motor forword Pin
  digitalWrite(IN4, LOW);   //Right Motor backword Pin
  analogWrite(speedL, reading);
  analogWrite(speedR, reading);
}

void right(int reading) {
  digitalWrite(IN1, HIGH);  //Left Motor backword Pin
  digitalWrite(IN2, LOW);   //Left Motor forword Pin
  digitalWrite(IN3, LOW);   //Right Motor forword Pin
  digitalWrite(IN4, HIGH);  //Right Motor backword Pin
  analogWrite(speedL, reading);
  analogWrite(speedR, reading);
}

// update LEDs based on the mode
void updateLEDs() {
  digitalWrite(LED_BLUETOOTH, isBluetoothMode ? HIGH : LOW);
  digitalWrite(LED_LINEFOLLOW, isBluetoothMode ? LOW : HIGH);
}
