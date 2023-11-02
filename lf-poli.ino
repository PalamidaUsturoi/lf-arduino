// -----Motor_Drive-----
const int IN1 = 5;
const int IN2 = 6;
const int IN3 = 9;
const int IN4 = 10;
const int EEP = 12;

// -----Sensor_Array-----
const int NO_SENSORS = 8; // 8
const int PAIRS = NO_SENSORS / 2;
const int S[NO_SENSORS] = { A4, A3, A5, A2, A6, A1, A7, A0 };
// const int S[NO_SENSORS] = { A5, A1 };
// A6 and A2 are the center sensors, A5 and A1 following

// -----Errors_and_Coef-----
const float kp_f = 2; // 2
const float kd_f = 5.5; // 6.25
const float ki_f = 0.00; // 0.02
const float kp_c = 4;
const float kd_c = 40;
const float ki_c = 0.03;
float err_old = 0;
float sum_err = 0;
const int threshold = 20;
const int threshold_getBack = 60;
// const int threshold_err = 10;

// -----Push_Buttons-----
const int BUTTON1 = 8;
const int BUTTON2 = 7;

float value[NO_SENSORS];
const int speed = 90;
const int speedCalibration = 100;
const int speedOneWheel = 90;

bool isRunning = false;
bool isCalibrating = false;
bool isDoing90 = false;
// int calibrationTime = 0; // 700

const String compensatingDir = "left";

int min_val[NO_SENSORS], max_val[NO_SENSORS];

const bool DEBUG = false;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EEP, OUTPUT);
  for (int i = 0; i < NO_SENSORS; i++) {
    pinMode(S[i], INPUT);
    min_val[i] = 1023;
    max_val[i] = 0;
  }
  isRunning = false;
  isCalibrating = false;
  isDoing90 = false;
  compensatingDir = "left";
  err_old = 0;
  sum_err = 0;
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  digitalWrite(EEP, HIGH);
  if ( DEBUG )
    Serial.begin(1000000);
}

void loop() {
  int b1, b2;
  b1 = digitalRead(BUTTON1);
  b2 = digitalRead(BUTTON2);

  if ( isDoing90 ) {
    if ( compensatingDir == "left" ) {
      moveMotor("left", -speedOneWheel);
      moveMotor("right", -speedOneWheel);
    }
    else {
      moveMotor("right", speedOneWheel);
      moveMotor("left", speedOneWheel);
    }
    
    value[2] = analogRead(S[2]);
    if ( value[2] < min_val[2] )
        min_val[2] = value[2];
      if ( value[1] > max_val[2] )
        max_val[2] = value[2];
    value[2] = mapf(value[2], min_val[2], max_val[2], 0, 100);

    value[NO_SENSORS - 3] = analogRead(S[NO_SENSORS - 3]);
    if ( value[NO_SENSORS - 3] < min_val[NO_SENSORS - 3] )
        min_val[NO_SENSORS - 3] = value[NO_SENSORS - 3];
      if ( value[NO_SENSORS - 3] > max_val[NO_SENSORS - 3] )
        max_val[NO_SENSORS - 3] = value[NO_SENSORS - 3];
    value[NO_SENSORS - 3] = mapf(value[NO_SENSORS - 3], min_val[NO_SENSORS - 3], max_val[NO_SENSORS - 3], 0, 100);

    if ( (compensatingDir == "left" && value[NO_SENSORS - 3] >= 100 - threshold_getBack) || 
         (compensatingDir == "right" && value[2] >= 100 - threshold_getBack) ) {
      // if ( compensatingDir == "left" ) {
      //   moveMotor("left", speedOneWheel);
      //   moveMotor("right", speedOneWheel);
      // }
      // else {
      //   moveMotor("right", -speedOneWheel);
      //   moveMotor("left", -speedOneWheel);
      // }
      // delay(100);

      // moveMotor("left", 0);
      // moveMotor("right", 0);
      // delay(50); // 50
      isDoing90 = false;
      isRunning = true;
      isCalibrating = false;
      // delay(2000);
      // int last = millis();
      // sum_err = err_old = 0;
      // while ( millis() - last < calibrationTime ) {
      //   for ( int i = 0; i < NO_SENSORS; i ++ ) {
      //     value[i] = analogRead(S[i]);
      //     if ( value[i] < min_val[i] )
      //       min_val[i] = value[i];
      //     if ( value[i] > max_val[i] )
      //       max_val[i] = value[i];
      //     value[i] = mapf(value[i], min_val[i], max_val[i], 0, 100);
      //   }

      //   float err;
      //   err = 0;
      //   for ( int i = 1; i < PAIRS; i ++ )
      //     err += (value[NO_SENSORS - i - 1] - value[i]) / PAIRS;

      //   float p = kp_c * err;
      //   float d = kd_c * (err - err_old);
      //   float i_cmp = ki_c * sum_err;
      //   err_old = err;
      //   sum_err += err;

      //   int speedLeft = speedCalibration + (p + d + i_cmp);
      //   int speedRight = speedCalibration - (p + d + i_cmp);

      //   moveMotor("left", speedLeft);
      //   moveMotor("right", -speedRight);
      // }
      moveMotor("left", 0);
      moveMotor("right", 0);
      // delay(2000);
    }
  }
  if ( isCalibrating ) {
    for ( int i = 0; i < NO_SENSORS; i ++ ) {
      int val = analogRead(S[i]);
      if ( val < min_val[i] )
        min_val[i] = val;
      if ( val > max_val[i] )
        max_val[i] = val;
    }
  }
  if ( isRunning ) {
    for ( int i = 0; i < NO_SENSORS; i ++ ) {
      value[i] = analogRead(S[i]);
      if ( value[i] < min_val[i] )
        min_val[i] = value[i];
      if ( value[i] > max_val[i] )
        max_val[i] = value[i];
      value[i] = mapf(value[i], min_val[i], max_val[i], 0, 100);
    }

    float err;
    err = 0;
    for ( int i = 1; i < PAIRS; i ++ )
      err += (value[NO_SENSORS - i - 1] - value[i]) / PAIRS;

    if ( (value[NO_SENSORS - 1] >= 100 - threshold) || (value[0] >= 100 - threshold) ) {
      moveMotor("left", speedOneWheel);
      moveMotor("right", -speedOneWheel);
      delay(20); // merem in fata pt spin
      // moveMotor("left", 0);
      // moveMotor("right", 0);
      // delay(1000);
      compensatingDir = "left";
      if ( value[NO_SENSORS - 1] >= 100 - threshold )
        compensatingDir = "right";
      isDoing90 = true;
      isRunning = false;
      isCalibrating = false;
    }
    else {
      float p = kp_f * err;
      float d = kd_f * (err - err_old);
      float i_cmp = ki_f * sum_err;
      err_old = err;
      sum_err += err;

      int speedLeft = speed + (p + d + i_cmp);
      int speedRight = speed - (p + d + i_cmp);

      moveMotor("left", speedLeft);
      moveMotor("right", -speedRight);
    }
  }
  else {
    if ( !b1 ) {
      isRunning = true;
      isCalibrating = false;
      isDoing90 = false;
    }
    else if ( !b2 ) {
      isRunning = false;
      isDoing90 = false;
      isCalibrating = true;
      moveMotor("left", 0);
      moveMotor("right", 0);
      for (int i = 0; i < NO_SENSORS; i++) {
        min_val[i] = 1023;
        max_val[i] = 0;
      }
      err_old = 0;
      sum_err = 0;
    }
  }
}

inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline void moveMotor(String motor, int speed) {
  int dir = 1;
  if (speed < 0) {
    dir = -1;
    speed *= -1;
  }
  if ( speed > 100 )
    speed = 100;
  speed = map(speed, 0, 100, 0, 255);
  if (dir == 1) {
    if (motor == "right") {
      analogWrite(IN1, speed);
      digitalWrite(IN2, LOW);
    } else {
      analogWrite(IN3, speed);
      digitalWrite(IN4, LOW);
    }
  } else {
    if (motor == "right") {
      digitalWrite(IN1, LOW);
      analogWrite(IN2, speed);
    } else {
      digitalWrite(IN3, LOW);
      analogWrite(IN4, speed);
    }
  }
}