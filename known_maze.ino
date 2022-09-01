
/*
  Types and Structures
*/
enum Status { ON, OFF };
enum Direction { BACKWARD, FORWARD, ROTATE_LEFT, ROTATE_RIGHT };
struct State { Direction direction; int amount; };

/*
  Global Variables
*/
// Pins
const int FRONT_SONAR_TRIG = 8,  FRONT_SONAR_ECHO = 5,
          LEFT_SONAR_TRIG  = 12, LEFT_SONAR_ECHO  = 11,
          LEFT_MOTOR_IN    = 7,  LEFT_MOTOR_OUT   = 6,  // Pins 3A and 4A
          RIGHT_MOTOR_IN   = 4,  RIGHT_MOTOR_OUT  = 9,  // Pins 1A and 2A

          SLOW_SPEED = 120, FAST_SPEED = 150;

const float MIN_FRONT_WALL_DISTANCE = 4.0,
            MAX_FRONT_WALL_DISTANCE = 10.0, // Other values 20 and 30 cm
            MIN_LEFT_WALL_DISTANCE  = 4.0,  // Other value 4.0
            MAX_LEFT_WALL_DISTANCE  = 6.5;  // Other value 6.5



int getDegree(const int degree) {
  int dt = 1250;
  switch (degree) {
    case 5:   return 150;
    case 45:  return dt / 2;
    case 360: return dt * 4;
    default:  return dt;
  };
}

// Path Variables
const int max_index = 15;
// Varible to store the entire path of the known maze
const State path [max_index] = {
  { FORWARD, -1 }, { ROTATE_RIGHT, 90 }, { FORWARD, -1 }, { ROTATE_RIGHT, 90 },
  { FORWARD, -1 }, { ROTATE_LEFT,  90 }, { FORWARD, -1 }, { ROTATE_RIGHT, 90 },
  { FORWARD, 5000 }, { ROTATE_RIGHT, 45 }, { FORWARD, 8000 }, { ROTATE_RIGHT, 45 },
  { FORWARD, -1 }, { ROTATE_LEFT,  90 }, { FORWARD, -1 }
};

int index = 0;                          // Index controlling the above array
State current_state = path[index];      // Rover's current state

int CURRENT_SPEED = SLOW_SPEED,
    LEFT_TILTs = 0, RIGHT_TILTs = 0;
long _LEFT_TILT_TIMER_ = millis();      // Left tilt decision timer

/*
  Methods
*/
boolean MazeFinished() {
  return (index >= (max_index - 1 ));
}

int Second_Smallest(int l[], int n) {
  int f = l[0], s = l[1];
  for (int i=0; i<n; i++) {
    if (l[i] < f) {
      s = f;
      f = l[i];
    } else if ((l[i] < s) && (l[i] != f)) s = l[i];
  }
  return s;
}

void SetNextState() {
  if (index <= (max_index - 1)) {
    index++;
    current_state = path[index];
  }
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
  
  int readings[3] = { 0, 0, 0 };
  readings[0] = ReadSonar(PIN_TRIG, PIN_ECHO);
  delay(100);
  readings[1] = ReadSonar(PIN_TRIG, PIN_ECHO);
  delay(100);
  readings[2] = ReadSonar(PIN_TRIG, PIN_ECHO);
  
  int m = Second_Smallest(readings, 3);
  
  return (m / 2.0) / 29.0;
}

boolean Obsticle(const int PIN_TRIG, const int PIN_ECHO, const float DISTANCE) {
  return ObsticleDistance(PIN_TRIG, PIN_ECHO) <= DISTANCE;
}

void Backward(const int PIN_IN, const int PIN_OUT, const int SPEED = CURRENT_SPEED) {
  analogWrite(PIN_IN, SPEED);
  analogWrite(PIN_OUT, 0);
}

void BackwardMotors() {
  Backward(LEFT_MOTOR_IN,  LEFT_MOTOR_OUT);
  Backward(RIGHT_MOTOR_IN, RIGHT_MOTOR_OUT);
}

void Forward(const int PIN_IN, const int PIN_OUT, const int SPEED = CURRENT_SPEED) {
  analogWrite(PIN_IN, 0);
  analogWrite(PIN_OUT, SPEED);
}

void ForwardMotors() {
  Forward(LEFT_MOTOR_IN,  LEFT_MOTOR_OUT);
  Forward(RIGHT_MOTOR_IN, RIGHT_MOTOR_OUT);
}

void LeftRotateMotors(const int degree) {
  Backward(LEFT_MOTOR_IN, LEFT_MOTOR_OUT);
  Forward(RIGHT_MOTOR_IN, RIGHT_MOTOR_OUT);
  delay(getDegree(degree));                                 // Need to Delay according to degree of rotation
}

void RightRotateMotors(const int degree) {
  Forward(LEFT_MOTOR_IN,   LEFT_MOTOR_OUT);
  Backward(RIGHT_MOTOR_IN, RIGHT_MOTOR_OUT);
  delay(getDegree(degree));                                 // Need to Delay according to degree of rotation
}

void Stop(const int PIN_IN, const int PIN_OUT) {
  digitalWrite(PIN_IN,  LOW);
  digitalWrite(PIN_OUT, LOW);
}

void StopMotors(const int _delay = 0) {
  Stop(LEFT_MOTOR_IN,  LEFT_MOTOR_OUT);
  Stop(RIGHT_MOTOR_IN, RIGHT_MOTOR_OUT);
  if (_delay != 0) delay(_delay);
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
  // Exit Game
  if (MazeFinished()) {
    delay(3000);
    StopMotors();
    return;
  }
  // Check for Obsticles on the front and left
  if (Obsticle(FRONT_SONAR_TRIG, FRONT_SONAR_ECHO, MIN_FRONT_WALL_DISTANCE)) {
    Serial.println("FRONT WALL");
    if (current_state.direction == FORWARD) {
      Serial.println("FRONT WALL - NEXT PATH");
      SetNextState();
    }
  } else if (float distance = ObsticleDistance(LEFT_SONAR_TRIG, LEFT_SONAR_ECHO)) {
    if ((index != 4) && (index != 6)) {
      if (((millis() - _LEFT_TILT_TIMER_) > 1000)) {
        if (distance < MIN_LEFT_WALL_DISTANCE) {
          if (RIGHT_TILTs < 3) {
            CURRENT_SPEED = FAST_SPEED;
            StopMotors();
            RightRotateMotors(5);
            StopMotors();
            CURRENT_SPEED = SLOW_SPEED;
            RIGHT_TILTs++;
            LEFT_TILTs = 0;
          }
        } else if (distance >= MAX_LEFT_WALL_DISTANCE) {
          if (LEFT_TILTs < 3) {
            CURRENT_SPEED = FAST_SPEED;
            StopMotors();
            LeftRotateMotors(5);
            StopMotors();
            CURRENT_SPEED = SLOW_SPEED;
            LEFT_TILTs++;
            RIGHT_TILTs = 0;
          }
        } else {
          LEFT_TILTs = 0;
          RIGHT_TILTs = 0;
        }
        _LEFT_TILT_TIMER_ = millis();
      }
    }
  }
  switch(current_state.direction) {
    case FORWARD:
      Serial.println(" - FORWARD");
      ForwardMotors();
      if (current_state.amount != -1) {
        delay(current_state.amount);
        SetNextState();
      }
      break;
    case ROTATE_LEFT:
      Serial.println(" - ROTATE LEFT");
      CURRENT_SPEED = FAST_SPEED;
      StopMotors(1000);
      LeftRotateMotors(current_state.amount);
      StopMotors(1000);
      SetNextState();
      CURRENT_SPEED = SLOW_SPEED;
      break;
    case ROTATE_RIGHT:
      Serial.println(" - ROTATE RIGHT");
      CURRENT_SPEED = FAST_SPEED;
      StopMotors(1000);
      RightRotateMotors(current_state.amount);
      StopMotors(1000);
      SetNextState();
      CURRENT_SPEED = SLOW_SPEED;
      break;
  };
}
