// ***THINGS TO TRY***
// Weighted Line
// Turn to 180 degrees -> Travel to ypos


#include "encoders.h"
#include "pid.h"
#include "line_sensors.h"
#include "kinematics.h"

// Pin definitions
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

// You may need to change these depending on how you wire
// in your line sensor.
#define LINE_LEFT_PIN   A4 //Pin for the left line sensor
#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor
#define LINE_RIGHT_PIN  A2 //Pin for the right line sensor

#define BUZZER 6

#define kp_home 0.50
#define ki_home 0.00
#define kd_home 0.50

#define kp_line 0.05
#define ki_line 0.00
#define kd_line 0.00

unsigned long start_time;
unsigned long prev_update;
unsigned long prev_move;
unsigned long time_stamp;
long prev_count_left;
long prev_count_right;

volatile float left_speed;
volatile float right_speed;

float home_theta;

int left_reading, centre_reading, right_reading;

int STATE;
bool setup_distance;
bool setup_check;

PID home_pid( kp_home, ki_home, kd_home );
PID line_pid( kp_line, ki_line, kd_line );

LineSensor line_left(LINE_LEFT_PIN); //Create a line sensor object for the left sensor
LineSensor line_centre(LINE_CENTRE_PIN); //Create a line sensor object for the centre sensor
LineSensor line_right(LINE_RIGHT_PIN); //Create a line sensor object for the right sensor

Kinematics pose;


// Remember, setup only runs once.
void setup() 
{
  // Initialise your other globals variables
  // and devices.

  setupEncoder0();
  setupEncoder1();

  line_left.calibrate();
  line_centre.calibrate();
  line_right.calibrate();

  STATE = 0;
  setup_distance = false;
  setup_check = false;

  start_time = millis();
  prev_update = millis(); 
  prev_move = millis();
  time_stamp = micros();
  prev_count_left = 0;
  prev_count_right = 0;

  left_speed = 0.0f;
  right_speed = 0.0f;

  left_reading = 0;
  centre_reading = 0;
  right_reading = 0;
  
  // Initialise the Serial communication
  Serial.begin( 9600 );
  delay(1000);
  Serial.println("***RESET***");
}



// Remmeber, loop is called again and again.
void loop() 
{
  unsigned long current_time = millis();
  unsigned long update_time = current_time - prev_update;
  unsigned long move_time = current_time - prev_move;

  if (update_time > 3) {
    prev_update = current_time;
    pose.update(count_left, count_right); // call an update to your kinematics at a time interval
  }


  if (move_time > 3) {
    prev_move = current_time;
    Serial.println(STATE);
    switch (STATE) {
      case 0: 
        FindLine();
        break;
      case 1:
        BangBang();
        //WeightedLine();
        break;
      case 2:
        RejoinLine();
        break;
      case 3:
        FaceHome();
        break;
      case 4:
        DriveHomeX();
        break;
      case 5:
        turn_angle_left(90.0f);
        break;
      case 6:
        DriveHomeY();
        break;
      case 7:
        leftMotor(0.0f);
        rightMotor(0.0f);
        Serial.println("Movement complete!");
      default:
        Serial.println("System Error, Unknown state!");
        break;
    }
  }

}

//Move forwards towards line.
void FindLine() {

  bool onLine = checkForLine();

  if (!onLine) {
    leftMotor(25.0f);
    rightMotor(24.0f);
  }
  else {
    leftMotor(0.0f);
    rightMotor(0.0f);

    //Set STATE to follow line.
    STATE = 1;
  }
}

//Initial check for line.
bool checkForLine() {

  bool onLine = false;

  left_reading = line_left.readCalibrated();
  centre_reading = line_centre.readCalibrated();
  right_reading = line_right.readCalibrated();

  if ( left_reading > 90 || centre_reading > 90 || right_reading > 90 ) {
    onLine = true;
  }

  return onLine;   
}


// Bang Bang Line Follower
void BangBang() {
  
  left_reading = line_left.readCalibrated();
  centre_reading = line_centre.readCalibrated();
  right_reading = line_right.readCalibrated();

  bool left_on_line = false;
  bool centre_on_line = false;
  bool right_on_line = false;
  
  if (left_reading > 80) left_on_line = true;
  if (centre_reading > 90) centre_on_line = true;
  if (right_reading > 80) right_on_line = true;

  if (centre_on_line) {
    leftMotor(24.0f);
    rightMotor(23.0f);
  }
  else if (left_on_line) {
    rightMotor(23.0f);
    leftMotor(-24.0f);
  }
  else if (right_on_line) {
    leftMotor(24.0f);
    rightMotor(-23.0f);
  }
  else {
    leftMotor(0.0f);
    rightMotor(0.0f);

    //Try to rejoin line
    STATE = 2;
  }

}


//Try to find line if lost
bool RejoinLine() {

  bool found_line = false;

  unsigned long current_time = millis();
  
  if (!setup_check) {
    start_time = millis();
    setup_check = true;
  }

  unsigned long elapsed_time = current_time - start_time;

  if (elapsed_time < 1000) {
    leftMotor(-20.0f);
    rightMotor(19.0f);
    found_line = checkForLine();
  }
  else if (elapsed_time < 2900) {
    leftMotor(20.0f);
    rightMotor(-19.0f);
    found_line = checkForLine();
  }
  else {
    leftMotor(0.0f);
    rightMotor(0.0f);
    analogWrite(BUZZER, 1);
    delay(2000);
    analogWrite(BUZZER, 0);
    STATE = 3;
  }
  
  if (found_line) {
    setup_check = false;
    STATE = 1;
  }
}


//Turn and face home.
void FaceHome() {
  float angle = pose.home_angle();
  turn_angle_right(angle);
}


//Drive distance from location to home.
void DriveHome() {

  float new_count_left, new_count_right;

  if (!setup_distance) {
    float distance = pose.home_distance();
    float count = (distance / (70.0f * PI)) * 1440.0f;
    new_count_left = count_left + count;
    new_count_right = count_right + count;
    setup_distance = true;
  }

  float speed = 20.0f;
  
  if (count_left < new_count_left) {
    leftMotor(speed + 1.0f);
  }
  else leftMotor(0.0f);

  if (count_right < new_count_right) {
    rightMotor(speed);
  }
  else rightMotor(0.0f);

  if (count_left >= new_count_left && count_right >= new_count_right) {
    setup_distance = false;
    STATE = 5;
  }
}


void DriveHomeX() {
  
  if (!setup_distance) {
    home_theta = pose.get_theta();
    setup_distance = true;
  }

  float theta_error = home_theta - pose.get_theta();
  int turn_pwm = 0;

  if (theta_error > 0){
    turn_pwm = -2;
  }
  else if (theta_error < 0) {
    turn_pwm = 2;
  }
  else turn_pwm = 0;

  int left_demand = 30 - turn_pwm;
  int right_demand = 30 + turn_pwm;

  leftMotor(left_demand);
  rightMotor(right_demand);

  if (abs(pose.get_xpos()) < 1) {
    setup_distance = false;
    STATE = 5;
  }

//  Serial.print( pose.get_xpos() );
//  Serial.print( ", " );
//  Serial.print( pose.get_ypos() );
//  Serial.print( ", " );
//  Serial.print( pose.get_theta() );
//  Serial.print( ", " );
//  Serial.println( home_theta );
  
}


void DriveHomeY() {
  
  if (!setup_distance) {
    home_theta = pose.get_theta();
    setup_distance = true;
  }

  float theta_error = home_theta - pose.get_theta();
  int turn_pwm = 0;

  if (theta_error > 0){
    turn_pwm = -2;
  }
  else if (theta_error < 0) {
    turn_pwm = 2;
  }
  else turn_pwm = 0;

  int left_demand = 30 - turn_pwm;
  int right_demand = 30 + turn_pwm;

  leftMotor(left_demand);
  rightMotor(right_demand);

  if (abs(pose.get_ypos()) < 1) {
    STATE = 7;
  }

//  Serial.print( pose.get_xpos() );
//  Serial.print( ", " );
//  Serial.print( pose.get_ypos() );
//  Serial.print( ", " );
//  Serial.print( pose.get_theta() );
//  Serial.print( ", " );
//  Serial.println( home_theta );
  
}


void turn_angle_right(float angle) {

  float new_count_left, new_count_right;

  if (!setup_distance) {
    float count = (((angle / 360.0f) * (140.0f * PI)) / (70.0f * PI)) * 1440.0f;
    new_count_left = count_left + count;
    new_count_right = count_right - count;
    setup_distance = true;
  }

  float speed = 20.0f;
  
  if (count_left < new_count_left) {
    leftMotor(speed + 1.0f);
  }
  else leftMotor(0.0f);

  if (count_right > new_count_right) {
    rightMotor(-speed);
  }
  else rightMotor(0.0f);

  if (count_left >= (new_count_left - 10) && count_right <= (new_count_right + 10)) {
    setup_distance = false;
    
    //Set STATE to drive towards home.
    STATE = 4;
  }
}


void turn_angle_left(float angle) {

  float new_count_left, new_count_right;

  if (!setup_distance) {
    float count = (((angle / 360.0f) * (140.0f * PI)) / (70.0f * PI)) * 1440.0f;
    new_count_left = count_left - count;
    new_count_right = count_right + count;
    setup_distance = true;
  }

  Serial.print(count_left);
  Serial.print( ", " );
  Serial.println(count_right);

  Serial.print(new_count_left);
  Serial.print( ", " );
  Serial.println(new_count_right);

  float speed = 20.0f;
  
  if (count_left > new_count_left) {
    leftMotor(-(speed + 1.0f));
  }
  else leftMotor(0.0f);

  if (count_right < new_count_right) {
    rightMotor(speed);
  }
  else rightMotor(0.0f);

  if (count_left <= new_count_left && count_right >= new_count_right) {
    setup_distance = false;
    STATE = 6;
  }
}


// ***MOTOR FUNCTIONS***
void leftMotor(float speed) {
  if (speed < -255.0f || speed > 255.0f) {
    Serial.println("Invalid Left Motor Speed.");
  }
  else {
    if (speed >= 0) digitalWrite( L_DIR_PIN, LOW );
    else digitalWrite( L_DIR_PIN, HIGH );

    speed = abs(speed);
    analogWrite( L_PWM_PIN, speed );
  }
}

void rightMotor(float speed) {
  if (speed < -255.0f || speed > 255.0f) {
    Serial.println("Invalid Right Motor Speed.");
  }
  else {
    if (speed >= 0) digitalWrite( R_DIR_PIN, LOW );
    else digitalWrite( R_DIR_PIN, HIGH );

    speed = abs(speed);
    analogWrite( R_PWM_PIN, speed );
  }
}




//UNUSED WEIGHTED LINE FOLLOWER
void WeightedLine() {
  left_reading = line_left.readCalibrated();
  centre_reading = line_centre.readCalibrated();
  right_reading = line_right.readCalibrated();

  if ( left_reading < 25 && centre_reading < 25 && right_reading < 25 ) {
    // Not on line
    leftMotor(0.0f);
    rightMotor(0.0f);

    /*//Set STATE to rejoin line
    STATE = 2;  */

    //Beep and Set STATE to face home.
    analogWrite(BUZZER, 1);
    delay(2000);
    analogWrite(BUZZER, 0);

    STATE = 3;
  }
  else {
    int total_reading = left_reading + centre_reading + right_reading;

    float left_weight = (float)left_reading / (float)total_reading;
    float centre_weight = (float)centre_reading / (float)total_reading;
    float right_weight = (float)right_reading / (float)total_reading;
  
    float line_centre = (left_weight * 1000) + (centre_weight * 2000) + (right_weight * 3000);
  
    line_centre -= 2000;

    float turn_pwm = line_pid.update( 0.0f, line_centre );

    int left_demand = 15 - turn_pwm;
    int right_demand = 15 + turn_pwm;

    left_demand = constrain( left_demand, -30, 30 );
    right_demand = constrain( right_demand, -30, 30 );

    leftMotor(left_demand);
    rightMotor(right_demand);
    
    /*Serial.print( turn_pwm );
    Serial.print( ", " );
    Serial.println( line_centre );*/
  }
  
}
