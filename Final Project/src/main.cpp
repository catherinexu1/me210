#include <Arduino.h>
#include <Metro.h> 
#include <NewPing.h> 
#include <Servo.h>

/* Ultrasonic sensors */ 
#define ECHO_LEFT               13
#define TRIG_LEFT               14   
#define ECHO_RIGHT              0 
#define TRIG_RIGHT              1  
#define ECHO_BACK               6 
#define TRIG_BACK               5   
#define ECHO_FRONT              7 
#define TRIG_FRONT              8  

#define YANK_PIN                15

#define SERVO_PIN               11
#define ANGLE_CLOSE             90
#define ANGLE_OPEN              0

#define ENABLE_PIN_LEFT         3
#define DIR_PIN_LEFT            4
#define ENABLE_PIN_RIGHT        17
#define DIR_PIN_RIGHT           16
#define ENABLE_PIN_FRONT        23
#define DIR_PIN_FRONT           22
#define ENABLE_PIN_BACK         9
#define DIR_PIN_BACK            10
#define ENABLE_PIN_WHEEL        20
#define DIR_PIN_WHEEL           21

#define STOP_DISTANCE           10
#define HALFWAY_DISTANCE        110
#define MAX_DISTANCE            4000

#define DISTANCE_TIME_INTERVAL  10
#define POWER_ON_TIME           120000
#define LOADING_TIME            3000
#define PRESS_TIME              3000
#define ROTATE_TIME             200
#define SHOOTING_TIME           5000
#define PING_DELAY_TIME         1
#define RUN_SPEED               200
#define FLYWHEEL_SPEED          1023
#define MOVE_DELAY              2000
#define HALFWAY_DISTANCE        110

typedef enum {
  STARTING, MOVE_FORWARD, MOVE_LEFT, MOVE_RIGHT, MOVE_BACKWARD, LOADING, SHOOTING, PRESS_BUTTON
} States_t; 

void moveForward();
void moveBackward();
void moveLeft(); 
void moveRight();
void stopMotors();
void pressButton();
void shootBalls();
void startFlywheel();
void stopFlywheel();
void print_state();
void rotateLeft();
void rotateRight();
void openServo();
void closeServo();
int pingDistance(NewPing sonar);
uint8_t TestDistanceTimerExpired();
void RespToDistanceTimerExpired(NewPing sonar);
uint8_t TestPowerTimerExpired();
void RespToPowerTimerExpired();

bool movingForward = true;
int shotJacobsLadder = 0;
bool movedHalfway = false;
int incomingByte = 0;
bool ballsLoaded = false;
unsigned int distance = 0;
States_t state = STARTING;
static Metro distanceTimer = Metro(DISTANCE_TIME_INTERVAL);
static Metro powerTimer = Metro(POWER_ON_TIME);
NewPing sonarLeft(TRIG_LEFT, ECHO_LEFT, MAX_DISTANCE); 
NewPing sonarRight(TRIG_RIGHT, ECHO_RIGHT, MAX_DISTANCE); 
NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE); 
NewPing sonarBack(TRIG_BACK, ECHO_BACK, MAX_DISTANCE); 
Servo myservo;

void setup() {
  Serial.begin(9600);
  pinMode(YANK_PIN, INPUT);
  pinMode(ENABLE_PIN_LEFT, OUTPUT);
  pinMode(ENABLE_PIN_RIGHT, OUTPUT);
  pinMode(ENABLE_PIN_FRONT, OUTPUT);
  pinMode(ENABLE_PIN_BACK, OUTPUT);
  pinMode(DIR_PIN_LEFT, OUTPUT);
  pinMode(DIR_PIN_RIGHT, OUTPUT);
  pinMode(DIR_PIN_FRONT, OUTPUT);
  pinMode(DIR_PIN_BACK, OUTPUT);
  analogWriteResolution(10);

  myservo.attach(SERVO_PIN);
  closeServo();
  stopFlywheel();
  while(digitalRead(YANK_PIN) == HIGH) {
    delay(100);
  }
  powerTimer.reset();
}

void print_state() {
  switch(state) {
    case STARTING: 
      Serial.println("STARTING state"); 
      break;
    case MOVE_FORWARD: 
      Serial.println("MOVE_FORWARD state"); 
      break;
    case MOVE_LEFT: 
      Serial.println("MOVE_LEFT state"); 
      break;
    case MOVE_RIGHT: 
      Serial.println("MOVE_RIGHT state"); 
      break;
    case MOVE_BACKWARD: 
      Serial.println("MOVE_BACKWARD state"); 
      break;
    case LOADING: 
      Serial.println("LOADING state"); 
      break;
    case SHOOTING: 
      Serial.println("SHOOTING state"); 
      break;
    case PRESS_BUTTON: 
      Serial.println("PRESS_BUTTON state"); 
      break;
  }
}

void loop() {
  if (TestPowerTimerExpired()) {
    stopMotors();
    stopFlywheel();
    exit(0);
  }

  print_state();
  /* Motor */ 
  switch (state) {
    case STARTING: 
      if(TestDistanceTimerExpired()) RespToDistanceTimerExpired(sonarFront);
      stopMotors();
      break;
    case MOVE_FORWARD:
      if(TestDistanceTimerExpired()) RespToDistanceTimerExpired(sonarFront);
      moveForward();
      break;
    case MOVE_BACKWARD:
      if(TestDistanceTimerExpired()) RespToDistanceTimerExpired(sonarBack);
      moveBackward();
      break;
    case MOVE_LEFT:
      if(TestDistanceTimerExpired()) RespToDistanceTimerExpired(sonarLeft);
      moveLeft();
      break;
    case MOVE_RIGHT:
      if(TestDistanceTimerExpired()) RespToDistanceTimerExpired(sonarRight);
      moveRight();
      break;
    case PRESS_BUTTON: 
      pressButton();
      break;
    case LOADING: 
      stopMotors();
      ballsLoaded = true;
      delay(LOADING_TIME);
      moveRight();
      delay(100);
      state = MOVE_FORWARD;
      break;
    case SHOOTING: 
      // stopMotors();
      if (shotJacobsLadder >= 2 && movedHalfway) {
        stopMotors();
        shootBalls();
        return;
      }
      if (shotJacobsLadder >= 2 && !movedHalfway) {
        state = MOVE_RIGHT;
      } else {
        moveLeft();
        delay(MOVE_DELAY);
        stopMotors();
        shootBalls();
      }
      break;
  }
}

uint8_t TestDistanceTimerExpired(void) {
  return (uint8_t) distanceTimer.check();
}

uint8_t TestPowerTimerExpired(void) {
  return (uint8_t) powerTimer.check();
}

void RespToDistanceTimerExpired(NewPing sonar) {
  distanceTimer.reset();
  distance = pingDistance(sonar);
  Serial.print(distance);
  Serial.println("cm");
  if (state == STARTING) {
    if (distance > 0) {
      state = MOVE_FORWARD;
    }
    return;
  }
  if (state == STARTING && distance > 0) {
    state = MOVE_FORWARD; 
    return;
  }
  if (state == MOVE_RIGHT && distance <= HALFWAY_DISTANCE) {
    movedHalfway = true;
    stopMotors();
    delay(100);
    state = SHOOTING;
    return;
  }
  if (distance > 0 && distance < STOP_DISTANCE) {
    if (state == MOVE_FORWARD && !ballsLoaded) {
      ;
    } else {
      delay(MOVE_DELAY);
    }
    stopMotors();
    delay(100);
    switch (state) {
      case MOVE_FORWARD:
        if (ballsLoaded) {
          state = SHOOTING;
          break;
        } else {
          state = MOVE_LEFT;
          break;
        }
      case MOVE_BACKWARD:
        state = PRESS_BUTTON;
        break;
      case MOVE_LEFT:
        state = MOVE_BACKWARD;
        break;
      default:
        break;
    }
  }
}

/* Get median distance of 5 attempts to normalize readings 
Borrowed from Sparki Library */
int pingDistance(NewPing sonar) {
  int attempts = 5;
  float distances [attempts];
  for(int i=0; i<attempts; i++){
    distances[i] = sonar.ping_cm();
    delay(PING_DELAY_TIME);
  }
  
  // sort them in order
  int i, j;
  float temp;
 
  for (i = (attempts - 1); i > 0; i--)
  {
    for (j = 1; j <= i; j++)
    {
      if (distances[j-1] > distances[j])
      {
        temp = distances[j-1];
        distances[j-1] = distances[j];
        distances[j] = temp;
      }
    }
  }
  
  // return the middle entry
  return int(distances[(int)ceil((float)attempts/2.0)]); 
}

void stopMotors() {
  analogWrite(ENABLE_PIN_LEFT, 0);
  analogWrite(ENABLE_PIN_RIGHT, 0);
  analogWrite(ENABLE_PIN_BACK, 0);
  analogWrite(ENABLE_PIN_FRONT, 0);
}

void moveForward() {
  analogWrite(ENABLE_PIN_LEFT, RUN_SPEED);
  analogWrite(ENABLE_PIN_RIGHT, RUN_SPEED);
  digitalWrite(DIR_PIN_LEFT, LOW);
  digitalWrite(DIR_PIN_RIGHT, HIGH);
  analogWrite(ENABLE_PIN_BACK, 0);
  analogWrite(ENABLE_PIN_FRONT, 0);
}

void moveBackward() {
  analogWrite(ENABLE_PIN_LEFT, RUN_SPEED);
  analogWrite(ENABLE_PIN_RIGHT, RUN_SPEED);
  digitalWrite(DIR_PIN_LEFT, HIGH);
  digitalWrite(DIR_PIN_RIGHT, LOW);
  analogWrite(ENABLE_PIN_BACK, 0);
  analogWrite(ENABLE_PIN_FRONT, 0);
}

void moveLeft() {
  analogWrite(ENABLE_PIN_LEFT, 0);
  analogWrite(ENABLE_PIN_RIGHT, 0);
  digitalWrite(DIR_PIN_BACK, LOW);
  digitalWrite(DIR_PIN_FRONT, HIGH);
  analogWrite(ENABLE_PIN_BACK, RUN_SPEED);
  analogWrite(ENABLE_PIN_FRONT, RUN_SPEED);
}

void moveRight() {
  analogWrite(ENABLE_PIN_LEFT, 0);
  analogWrite(ENABLE_PIN_RIGHT, 0);
  digitalWrite(DIR_PIN_BACK, HIGH);
  digitalWrite(DIR_PIN_FRONT, LOW);
  analogWrite(ENABLE_PIN_BACK, RUN_SPEED);
  analogWrite(ENABLE_PIN_FRONT, RUN_SPEED);
}

void startFlywheel() {
  analogWrite(ENABLE_PIN_WHEEL, FLYWHEEL_SPEED);
}

void stopFlywheel() {
  analogWrite(ENABLE_PIN_WHEEL, 0); 
}

void pressButton() {
  moveLeft();
  delay(PRESS_TIME);
  state = LOADING;
}

void shootBalls() {
  // rotateLeft(); 
  // delay(ROTATE_TIME); 
  // stopMotors();
  startFlywheel();
  openServo(); 
  delay(SHOOTING_TIME);
  closeServo();
  stopFlywheel();
  moveRight();
  delay(500);
  stopMotors();
  shotJacobsLadder += 1;
  // rotateRight();
  // delay(ROTATE_TIME);
  // stopMotors();
  if (shotJacobsLadder >= 3) {
    movedHalfway = false;
    state = MOVE_LEFT;
    return;
  }
  state = MOVE_BACKWARD;
  ballsLoaded = false;
}

void rotateLeft() {
  analogWrite(ENABLE_PIN_LEFT, 1023);
  analogWrite(ENABLE_PIN_RIGHT, 1023);
  digitalWrite(DIR_PIN_LEFT, HIGH);
  digitalWrite(DIR_PIN_RIGHT, HIGH);
  analogWrite(ENABLE_PIN_BACK, 0);
  analogWrite(ENABLE_PIN_FRONT, 0);
}

void rotateRight() {
  analogWrite(ENABLE_PIN_LEFT, 1023);
  analogWrite(ENABLE_PIN_RIGHT, 1023);
  digitalWrite(DIR_PIN_LEFT, LOW);
  digitalWrite(DIR_PIN_RIGHT, LOW);
  analogWrite(ENABLE_PIN_BACK, 0);
  analogWrite(ENABLE_PIN_FRONT, 0);
}

void openServo() {
  myservo.write(ANGLE_OPEN);
}

void closeServo() {
  myservo.write(ANGLE_CLOSE);
}