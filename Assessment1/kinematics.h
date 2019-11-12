
#ifndef _Kinematics
#define _Kinematics_h

#define PI 3.1415926535897932384626433832795

//You may want to use some/all of these variables
const float WHEEL_DIAMETER              = 70.0f; //mm
const float WHEEL_RADIUS                = 35.0f; //mm
const float WHEEL_SEPERATION            = 141.0f;
const float GEAR_RATIO                  = 1.0f/120.0f;
const float COUNTS_PER_SHAFT_REVOLUTION = 12.0f;
const float COUNTS_PER_WHEEL_REVOLUTION =  1440.0f;
const float COUNTS_PER_MM               = (1 / (WHEEL_DIAMETER * PI)) * COUNTS_PER_WHEEL_REVOLUTION;


// Build up your Kinematics class.
class Kinematics
{
  public:
    
    Kinematics();   // Constructor, required.

    // Write your method functions:
    // ...
    void update(long count_left, long count_right);  // should calucate an update to pose.
    float home_angle();
    float home_distance();
    void reset_theta();
    float get_theta();
    float get_xpos();
    float get_ypos();
    
  private:
    
    //Private variables and methods go here
    float xpos;
    float ypos;
    float theta;

    long old_count_left;
    long old_count_right;
    
};


// Required constructor.  Initialise variables.
Kinematics::Kinematics() {
  xpos = 0;
  ypos = 0;
  theta = 0;

  old_count_left = 0;
  old_count_right = 0;

}

void Kinematics :: update(long count_left, long count_right) {

  long change_left = count_left - old_count_left;
  long change_right = count_right - old_count_right;

  float left_distance = (float)change_left / COUNTS_PER_MM;
  float right_distance = (float)change_right / COUNTS_PER_MM;

  float avg_distance = (left_distance + right_distance) / 2.0f;
  
  xpos = xpos + avg_distance * cos(theta);
  ypos = ypos + avg_distance * sin(theta); 

  theta = theta + (left_distance - right_distance) / WHEEL_SEPERATION;

  /*Serial.print( xpos );
  Serial.print( ", " );
  Serial.print( ypos );
  Serial.print( ", " );
  Serial.println( theta );*/

  old_count_left = count_left;
  old_count_right = count_right;
  return;
}


float Kinematics :: home_angle() {
  float angle = (- theta / (2 * PI) * 360) + 180 - atan2(ypos, xpos);
  //float angle = atan2(ypos, xpos);
  return angle;
}


float Kinematics :: home_distance() {
  float distance = sqrt(xpos * xpos + ypos * ypos);
  return distance;
}


void Kinematics :: reset_theta() {
  theta = 0;
}


float Kinematics :: get_theta() {
  return theta;
}

float Kinematics :: get_xpos() {
  return xpos;
}

float Kinematics :: get_ypos() {
  return ypos;
}



#endif
