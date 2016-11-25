#include <QTRSensors.h>

// execution parameters
int rotation_speed = 150;
int distance_threshold = 15;
int line_following_threshold = 50;  // milliseconds between each direction correction

// QTR-8A Reflectance Sensor Array
#define PWM_1 5  // left motor speed control
#define PWM_2 6  // right motor speed control
#define in1_1 7  // upside wire of left motor
#define in1_2 8  // downside wire of left motor
#define in2_1 9  // upside wire of right motor
#define in2_2 10  // downside wire of right motor

#define NUM_SENSORS 6  
#define NUM_SAMPLES_PER_SENSOR 4  
#define EMITTER_PIN QTR_NO_EMITTER_PIN  // emitter is controlled by digital pin 2

QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5}, 
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

int qtra_mid = 0, qtra_max = 0, qtra_min = 0;

// HC-SR04 Ultrasonic distance sensor
#define trig_pin 13
#define echo_pin 12

void flash_DS3(int num_of_times) {
  int i;
  for (i = 0; i < num_of_times; i++) {
    digitalWrite(13, HIGH);
    delay(250);
    digitalWrite(13, LOW);
    delay(250);
  }
}

long get_distance() {
  long hc_duration = 0, hc_distance = 0;
  // initialize trigger input
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(5);
  
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  // linsten for echo pulse output
  hc_duration = pulseIn(echo_pin, HIGH);
  hc_distance = hc_duration / 58;  // distance in centimeter with sound speed 344 m/s

  return hc_distance;
}

void go_forward_with_speed(int motor_speed) {
  analogWrite(PWM_1, motor_speed);
  analogWrite(PWM_2, motor_speed);
  
  digitalWrite(in1_1, HIGH);
  digitalWrite(in1_2, LOW);
  digitalWrite(in2_1, HIGH);
  digitalWrite(in2_2, LOW);
}

void make_turn(int left_speed, int right_speed) {
  analogWrite(PWM_1, left_speed);
  analogWrite(PWM_2, right_speed);
  
  digitalWrite(in1_1, HIGH);
  digitalWrite(in1_2, LOW);
  digitalWrite(in2_1, HIGH);
  digitalWrite(in2_2, LOW);
}

void rotate_cw_with_speed(int motor_speed) {
  analogWrite(PWM_1, motor_speed);
  analogWrite(PWM_2, motor_speed);
  
  digitalWrite(in1_1, HIGH);
  digitalWrite(in1_2, LOW);
  digitalWrite(in2_1, LOW);
  digitalWrite(in2_2, HIGH);
}

void stop_motor() {
  digitalWrite(in1_1, LOW);
  digitalWrite(in1_2, LOW);
  digitalWrite(in2_1, LOW);
  digitalWrite(in2_2, LOW);
}

void start_following_line() {  // following line until reaching a horizontal black line
  while (1) {
    go_forward_with_speed(255);
    delay(line_following_threshold);
    
    qtra.read(sensorValues);
    int left_mid = (sensorValues[0] + sensorValues[1] + sensorValues[2]) / 3,
      right_mid = (sensorValues[3] + sensorValues[4] + sensorValues[5]) / 3;
    if (left_mid > right_mid) {
      make_turn(255, 150);  // turn right
      delay(left_mid - right_mid);
    }
    if (left_mid < right_mid) {
      make_turn(150, 255);  // turn left
      delay(right_mid - left_mid);
    }

    if ((sensorValues[0] > qtra_mid) && (sensorValues[5] > qtra_mid)) {
      stop_motor();
      return;
    }
  }
}

void setup() {
  digitalWrite(13, HIGH);
  Serial.begin(9600);
  
  // initialize motors
  pinMode(PWM_1, OUTPUT);
  pinMode(PWM_2, OUTPUT);
  pinMode(in1_1, OUTPUT);
  pinMode(in1_2, OUTPUT);
  pinMode(in2_1, OUTPUT);
  pinMode(in2_2, OUTPUT);

  analogWrite(PWM_1, 255);
  analogWrite(PWM_2, 255);
  digitalWrite(in1_1, LOW);
  digitalWrite(in1_2, LOW);
  digitalWrite(in2_1, LOW);
  digitalWrite(in2_2, LOW);

  // initialize distance sensor
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);

  // calibrate IR sensors
  int i, j;
  for (i = 0; i < 400; i++) {
    qtra.read(sensorValues);
    for (j = 0; j < NUM_SENSORS; j++) {
      if (sensorValues[j] > qtra_max) {
        qtra_max = sensorValues[j];
      }
      if (sensorValues[j] < qtra_min) {
        qtra_min = sensorValues[j];
      }
    }
  }
  qtra_mid = (qtra_max + qtra_min) / 2;
  
  digitalWrite(13, LOW);
}

void loop() {
  start_following_line();
  flash_DS3(1);

  // reaches a horizontal black line: check if parking space is empty
  int hc_distance = get_distance();
  while (hc_distance < distance_threshold) {
    start_following_line();
    flash_DS3(1);
  }
  flash_DS3(1);
  
  // reaches an empty parking space: turning right to face the space
  rotate_cw_with_speed(rotation_speed);
  while (1) {
    qtra.read(sensorValues);
    int black_count = 0, i;
    for (i = 0; i < NUM_SENSORS; i++) {
      if (sensorValues[i] > qtra_mid) {
        black_count++;
      }
    }
    if (black_count >= 3) {
      break;
    }
  }
  flash_DS3(2);

  // go into the parking space by following direction line
  start_following_line();
  flash_DS3(3);

  exit(0);
}
