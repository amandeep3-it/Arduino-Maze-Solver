
/*
  Global Variables
*/

// Pins of the HC-SR04 Sonar Sensors and N20 DC Gear Motors
const int FRONT_SONAR_TRIG = 8,  FRONT_SONAR_ECHO = 5,
          LEFT_SONAR_TRIG  = 12, LEFT_SONAR_ECHO  = 11,
          LEFT_MOTOR_IN    = 7,  LEFT_MOTOR_OUT   = 6,
          RIGHT_MOTOR_IN   = 4,  RIGHT_MOTOR_OUT  = 9,

          // Slow speed for forward/backward functionality and Mid speed for rotation
          SLOW_SPEED = 140,    MID_SPEED = 150;

// Distances for front and left wall
const float MIN_FRONT_DISTANCE = 5.0,
            MAX_FRONT_DISTANCE = 10.0,
            MIN_LEFT_DISTANCE  = 4.0,
            MAX_LEFT_DISTANCE  = 6.5,
            FAR_LEFT_DISTANCE  = 15;

int CURRENT_SPEED = SLOW_SPEED;

long _LEFT_TILT_TIMER_ = millis();      // Left Tilt Decision Timer



/*
  Methods
*/

// Pre-defined rotation durations
int getDegreeDuration(const int degree) {
  const int dt = 1150;
  switch (degree) {
    case 5:   return 150;
    case 10:  return 300;
    case 45:  return dt / 2;
    case 360: return dt * 4;
    default:  return dt;
  };
}

int ReadSonar(const int PIN_TRIG, const int PIN_ECHO) {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  return pulseIn(PIN_ECHO, HIGH);
}

float ObsticleDistance(const int PIN_TRIG, const int PIN_ECHO) {
  return (ReadSonar(PIN_TRIG, PIN_ECHO) / 2.0) / 29.0;
}

boolean Obsticle(const int PIN_TRIG, const int PIN_ECHO, const float DISTANCE) {
  return ObsticleDistance(PIN_TRIG, PIN_ECHO) <= DISTANCE;
}

void Backward(const int PIN_IN, const int PIN_OUT, const int SPEED = CURRENT_SPEED) {
  analogWrite(PIN_IN,  SPEED);
  analogWrite(PIN_OUT, 0);
}

void BackwardMotors() {
  Backward(LEFT_MOTOR_IN,  LEFT_MOTOR_OUT);
  Backward(RIGHT_MOTOR_IN, RIGHT_MOTOR_OUT);
}

void Forward(const int PIN_IN, const int PIN_OUT, const int SPEED = CURRENT_SPEED) {
  analogWrite(PIN_IN,  0);
  analogWrite(PIN_OUT, SPEED);
}

void ForwardMotors() {
  Forward(LEFT_MOTOR_IN,  LEFT_MOTOR_OUT);
  Forward(RIGHT_MOTOR_IN, RIGHT_MOTOR_OUT);
}

void LeftRotateMotors(const int degree) {
  Backward(LEFT_MOTOR_IN, LEFT_MOTOR_OUT);
  Forward(RIGHT_MOTOR_IN, RIGHT_MOTOR_OUT);
  delay(getDegreeDuration(degree));               // Delay according to degree of rotation
}

void RightRotateMotors(const int degree) {
  Forward(LEFT_MOTOR_IN,   LEFT_MOTOR_OUT);
  Backward(RIGHT_MOTOR_IN, RIGHT_MOTOR_OUT);
  delay(getDegreeDuration(degree));               // Delay according to degree of rotation
}

void Stop(const int PIN_IN, const int PIN_OUT) {
  digitalWrite(PIN_IN,  LOW);
  digitalWrite(PIN_OUT, LOW);
}

void StopMotors(const int _dur = 0) {
  Stop(LEFT_MOTOR_IN,  LEFT_MOTOR_OUT);
  Stop(RIGHT_MOTOR_IN, RIGHT_MOTOR_OUT);
  if (_dur != 0) delay(_dur);                     // Delay if duration provided
}



/*
  Running Code
*/

void setup() {
  Serial.begin(9600);
  pinMode(FRONT_SONAR_TRIG, OUTPUT);
  pinMode(FRONT_SONAR_ECHO, INPUT);
  pinMode(LEFT_SONAR_TRIG,  OUTPUT);
  pinMode(LEFT_SONAR_ECHO,  INPUT);
  pinMode(LEFT_MOTOR_IN,    OUTPUT);
  pinMode(LEFT_MOTOR_OUT,   OUTPUT);
  pinMode(RIGHT_MOTOR_IN,   OUTPUT);
  pinMode(RIGHT_MOTOR_OUT,  OUTPUT);
  StopMotors();
}

void loop() {

  // Check if the wall at the front of the rover is detected
  if (Obsticle(FRONT_SONAR_TRIG, FRONT_SONAR_ECHO, MIN_FRONT_DISTANCE)) {
    StopMotors(1000);
    // Check if the wall is at the left of the rover
    // and make decision of left or right rotation
    if (float distance = ObsticleDistance(LEFT_SONAR_TRIG, LEFT_SONAR_ECHO)) {
      CURRENT_SPEED = MID_SPEED;
      StopMotors(1000);
      if (distance < FAR_LEFT_DISTANCE) RightRotateMotors(90);
      else LeftRotateMotors(90);
      StopMotors(1000);
      CURRENT_SPEED = SLOW_SPEED;
    }
  }

  // Trace the left wall
  if (float distance = ObsticleDistance(LEFT_SONAR_TRIG, LEFT_SONAR_ECHO)) {
    // Make a decision every 1 second
    if (((millis() - _LEFT_TILT_TIMER_) > 1000)) {
      boolean done = false;

      // Tilt the rover away from the wall if too close
      if (distance < MIN_LEFT_DISTANCE) {
        CURRENT_SPEED = MID_SPEED;
        StopMotors();
        RightRotateMotors(5);
        StopMotors(100);
        CURRENT_SPEED = SLOW_SPEED;
        done = true;
      }

      // Tilt the rover towards from the wall if too far
      if (!done && (distance > MAX_LEFT_DISTANCE) && (distance < FAR_LEFT_DISTANCE)) {
        CURRENT_SPEED = MID_SPEED;
        StopMotors();
        LeftRotateMotors(5);
        StopMotors(100);
        CURRENT_SPEED = SLOW_SPEED;
        done = true;
      }

      // Tilt the rover at 45° if the left wall is no longer detectable
      if (!done && (distance > FAR_LEFT_DISTANCE)) {
        CURRENT_SPEED = MID_SPEED;
        delay(800);
        StopMotors(1000);
        LeftRotateMotors(45);
        StopMotors(1000);
        CURRENT_SPEED = SLOW_SPEED;
      }

      _LEFT_TILT_TIMER_ = millis();
    }
  }

  ForwardMotors();
}
