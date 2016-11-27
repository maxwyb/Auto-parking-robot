#include <QTRSensors.h>

// execution parameters
int line_following_speed = 160;
int rotation_speed = 180;  /* experimental threshold 180 on foam board to start rotating from at rest */
int distance_threshold = 30;
int line_following_threshold = 20;  // milliseconds between each direction correction

// motors
#define PWM_1 5  // left motor speed control
#define PWM_2 6  // right motor speed control
#define in1_1 7  // upside wire of right motor
#define in1_2 8  // downside wire of right motor
#define in2_1 9  // upside wire of left motor
#define in2_2 10  // downside wire of left motor

// QTR-8A Reflectance Sensor Array
#define NUM_SENSORS 6  
#define NUM_SAMPLES_PER_SENSOR 4  
#define EMITTER_PIN QTR_NO_EMITTER_PIN  // emitter is controlled by digital pin 2

QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5}, 
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

int qtra_mid = -1, qtra_max = 0, qtra_min = 1023;

// HC-SR04 Ultrasonic distance sensor
#define trig_pin 13
#define echo_pin 12

void flash_DS2(int num_of_times) {
  int i;
  for (i = 0; i < num_of_times; i++) {
    digitalWrite(13, HIGH);
    delay(250);
    digitalWrite(13, LOW);
    delay(250);
  }
}

void print_IR_sensor_values() {
    Serial.print(sensorValues[0]);
    Serial.print(" ");
    Serial.print(sensorValues[1]);
    Serial.print(" ");
    Serial.print(sensorValues[2]);
    Serial.print(" ");
    Serial.print(sensorValues[3]);
    Serial.print(" ");
    Serial.print(sensorValues[4]);
    Serial.print(" ");
    Serial.println(sensorValues[5]);
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

  Serial.print("get_distance called: ultrasonic_distance = ");
  Serial.println(hc_distance);
  return hc_distance;
}

void go_forward_with_speed(int motor_speed) {
  analogWrite(PWM_2, motor_speed);
  analogWrite(PWM_1, motor_speed);

  digitalWrite(in2_1, HIGH);
  digitalWrite(in2_2, LOW);
  digitalWrite(in1_1, HIGH);
  digitalWrite(in1_2, LOW);
}

void make_turn(int left_speed, int right_speed) {  // turning strategy 1: both motor spinning
  analogWrite(PWM_2, left_speed);
  analogWrite(PWM_1, right_speed);

  if (left_speed == 0) {
    digitalWrite(in2_1, LOW);
    digitalWrite(in2_2, LOW);
  } else {
    digitalWrite(in2_1, HIGH);
    digitalWrite(in2_2, LOW);
  }

  if (right_speed == 0) {
    digitalWrite(in1_1, LOW);
    digitalWrite(in1_2, LOW);
  } else {
    digitalWrite(in1_1, HIGH);
    digitalWrite(in1_2, LOW);
  }
}

void rotate_cw_with_speed(int motor_speed) {  // turning strategy 2: self-rotation
  analogWrite(PWM_2, motor_speed);
  analogWrite(PWM_1, motor_speed);

  digitalWrite(in2_1, HIGH);
  digitalWrite(in2_2, LOW);
  digitalWrite(in1_1, LOW);
  digitalWrite(in1_2, HIGH);
}

void stop_motor() {
  digitalWrite(in2_1, LOW);
  digitalWrite(in2_2, LOW);
  digitalWrite(in1_1, LOW);
  digitalWrite(in1_2, LOW);
}

void start_following_line() {  // following line until reaching a horizontal black line
  // execution parameters
  /*
   * pairs of usually correct line following: 
   * turning_delay_factor = 0.4; comp_offset = 250
   * turning_delay_factor = 0.4; comp_offset = 285
   */
  double turning_delay_factor = 0.4;  // correct line following threshold: 0.5
  int comp_offset = 250;  
    
  go_forward_with_speed(line_following_speed);
  delay(line_following_threshold);
  
  while (1) {
    
    qtra.read(sensorValues);
    int left_mid = (sensorValues[0] + sensorValues[1] + sensorValues[2]) / 3,
      right_mid = (sensorValues[3] + sensorValues[4] + sensorValues[5]) / 3;
    if (left_mid > right_mid) {
      make_turn(line_following_speed, 0);  // turn right
      int delay_time = turning_delay_factor * (left_mid - right_mid);
      Serial.println("--line following: turning right to offset.");
      Serial.print("--line following: running offset time = ");
      Serial.println(delay_time);
      delay(delay_time);
    }
    if (left_mid < right_mid) {
      make_turn(0, line_following_speed);  // turn left
      int delay_time = turning_delay_factor * (right_mid - left_mid);
      Serial.println("--line following: turning left to offset.");
      Serial.print("--line following: running offset time = ");
      Serial.println(delay_time);
      delay(delay_time);
    }

    
    // determine black tape to stop: approach 1
    if ((sensorValues[0] > (qtra_mid + comp_offset)) && (sensorValues[5] > (qtra_mid + comp_offset))) {  
      Serial.println("--line following: detect horizontal black tape; stop.");
      Serial.print("--current IR sensor values: ");
      print_IR_sensor_values();
      stop_motor();
      return;
    }
    
    /*
    // determine black tape to stop: approach 2
    int comparison_offset = 100;  // FIXIT: temporary solution
    int black_count = 0, i;
    for (i = 0; i < NUM_SENSORS; i++) {
      if (sensorValues[i] > (qtra_mid + comparison_offset)) {
        black_count++;
      }
    }
    if (black_count >= 5) {
      Serial.println("--line following: detect horizontal black tape; stop.");
      Serial.print("--current IR sensor values: ");
      print_IR_sensor_values();
      stop_motor();
      return;
    }
    */
  }
}

void setup() {
  pinMode(13, OUTPUT);
  
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
  Serial.println("setup: starting IR sensor calibration.");
  for (i = 0; i < 2000; i++) {
    qtra.read(sensorValues);
    for (j = 0; j < 6; j++) {  
      if (sensorValues[j] > qtra_max) {
        qtra_max = sensorValues[j];
      }
      if (sensorValues[j] < qtra_min) {
        qtra_min = sensorValues[j];
      }
    }
  }
  qtra_mid = (qtra_max + qtra_min) / 2;
  Serial.print("qtra_max = ");
  Serial.print(qtra_max);
  Serial.print("; qtra_min = ");
  Serial.print(qtra_min);
  Serial.print("; qtra_mid = ");
  Serial.println(qtra_mid);
  Serial.println("setup: finish IR sensor calibration.");
  
  digitalWrite(13, LOW);
  flash_DS2(10);
  delay(3000);
}

void loop() {
  flash_DS2(1);
  // execution parameters (for detecting tape during rotation)
  /* 
   * pairs of sually correct rotation detection:
   * parking space 1, RV6 lounge sunny: sensor_count_threshold = 5, comp_offset = 100
   * (testing) sensor_count_threshold = 5, comp_offset = 75
   */
  int sensor_count_threshold = 5;
  int comp_offset = 100;
  
  Serial.println("Start following line.");
  start_following_line();
  Serial.println("reached horizontal black tape.");
  //flash_DS2(1);

  // reaches a horizontal black line: check if parking space is empty
  int hc_distance = get_distance();
  while (hc_distance < distance_threshold) {
    Serial.println("parking space not empty. Proceeding.");
    delay(2000);
    start_following_line();
    //flash_DS2(1);
    hc_distance = get_distance();
  }
  Serial.println("parking space empty! Doing rotation.");
  //flash_DS2(1);
  delay(2000);
  
  // reaches an empty parking space: turning right to face the space
  rotate_cw_with_speed(rotation_speed);
  delay(500);
  while (1) {
    qtra.read(sensorValues);
    int black_count = 0, i;
    for (i = 0; i < NUM_SENSORS; i++) {
      if (sensorValues[i] > (qtra_mid + comp_offset)) {
        black_count++;
      }
    }
    if (black_count >= sensor_count_threshold) {
      Serial.println("--rotation: detect black tape.");
      Serial.print("--current IR sensor values: ");
      print_IR_sensor_values();
      break;
    }
  }
  stop_motor();
  //flash_DS2(2);
  delay(2000);

  // go into the parking space by following direction line
  Serial.println("Rotation done. Going into parking space by following line.");
  start_following_line();

  Serial.println("Parking done! exit.");
  flash_DS2(3);
  exit(0);
}

