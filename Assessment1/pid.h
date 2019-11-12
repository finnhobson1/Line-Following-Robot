#ifndef _PID_h
#define _PID_h
#include <stdint.h>

/*Here, the definition of the PID class begins. This is indicated by the keyword: "class"
This is a general description of the data and functions that the class contains. 
To use a class, we must make a specific instance of the class by declaring it into the same way we declare a variable. 
For example, to create a version of the PID class, in our main file we might write:

PID LeftWheelPID;
PID RightWheelPID;

This will create two instances of the PID class; one for the left wheel and one for the right wheel. 
Each class will have a full copy of all the variables and functions defined for that particular class.
*/ 

class PID {
  /* Public functions and variables are defined here. A public function / variable can be accessed from outside 
   * the class. 
   * For example, once we have made an instance of the PID class, we can call the update function by writing:
   * 
   * LeftWheelPID.update();
   * 
   * Note that this will only update the LeftWheelPID - RightWheelPID will not be updated unless we also call 
   * RightWheelPID.update()
   */
  public:

    PID(float P, float I, float D);                 // This is the class constructor. It is called whenever we create an instance of the PID class 
    void setGains(float P, float I, float D );      // This function updates the values of the gains
    void reset();                                   // This function resets any stored values used by the integral or derative terms
    float update(float demand, float measurement);  // This function calculates the PID control signal. It should be called in a loop
    void printComponents();                        // This function prints the individual components of the control signal and can be used for debugging
    void setMax(float  newMax);                     // This function sets the maximum output the controller can ask for
    void setDebug(bool state);                      // This function sets the debug flag;
    void printResponse();                          // This function prints the ratio of input to output in a way that is nicely interpreted by the Serial plotter
    void setShowResponse(bool state);             // This functions set the show_response flag

  /* Private functions and variables are defined here. These functions / variables cannot be accessed from outside the class.
   * For example, if we try to set the value of Kp in the file "Romi.h", we will get an error (Try it out!) 
   * You might want to move some into public space when you are debugging ;)
   */
  private:

    //Control gains
    float Kp; //Proportional
    float Ki; //Integral
    float Kd; //Derivative

    //We can use this to limit the output to a certain value
    float max_output; 

    //Output components
    //These are used for debugging purposes
    float Kp_output; 
    float Ki_output;
    float Kd_output;
    float output_signal;

    //Values to store between updates().
    float last_demand;      //For storing the previous input
    float last_measurement; //For storing the last measurement
    float last_error;       //For calculating the derivative term
    float integral_error;   //For storing the integral of the error
    long last_millis;       //To track elapsed_time
    bool debug;             //This flag controls whether we print the contributions of each component when update is called
    bool show_response;     // This flag controls whether we print the response of the controller on each update
    
};

/*
 * Class constructor
 * This runs whenever we create an instance of the class
 */
 PID::PID(float P, float I, float D)
{
  //Store the gains
  setGains(P, I, D);
  
  // Initialise key variables.
  Kp_output     = 0;
  Ki_output     = 0;
  Kd_output     = 0;
  output_signal = 0;

  max_output        = 255;
  last_demand       = 0;
  last_measurement  = 0;
  last_error        = 0;
  integral_error    = 0;
  debug             = false;
  show_response     = false;
  last_millis       = millis();
  
}

/*
 * This function prints the individual contributions to the total contol signal
 * You can call this yourself for debugging purposes, or set the debug flag to true to have it called
 * whenever the update function is called.
 */
void PID::printComponents() {
  Serial.print(Kp_output);
  Serial.print(",");
  Serial.print(Kd_output);
  Serial.print(",");
  Serial.print(Ki_output);
  Serial.print(",");
  Serial.print(output_signal);
  Serial.print("\n");
}

/*
 * This function sets the gains of the PID controller
 */
void PID::setGains(float P, float I, float D) {
  Kp = P;
  Ki = I;
  Kd = D;
}

/*
 * This is the update function. 
 * This function should be called repeatedly. 
 * It takes a measurement of a particular variable (ex. Position, speed, heading) and a desired value for that quantity as input
 * It returns an output; this can be sent directly to the motors, 
 * combined with other control outputs
 * or sent as input to another controller
 */
float PID::update(float demand, float measurement) {
  //Calculate how much time (in milliseconds) has passed since the last update call
  // Watch out for when time_delta might equal 0
  long time_now = millis();
  int time_delta = time_now - last_millis;
  last_millis = time_now;

  /*
   * ================================
   * Your code goes implementation of a PID controller should go here
   * ================================
   */

  //This represents the error term
  // Decide what your error signal is (demand vs measurement)
  float error;
  error = demand - measurement;   
  
  //This represents the error derivative
  // Calculate the change in your error between update()
  float error_delta;
  error_delta = (error - last_error)/float(time_delta);
  last_error = error;

  // This represents the error integral.
  // Integrate error over time.
  integral_error += error * float(time_delta);

  //Attenuate above error components by gain values.
  Kp_output = Kp * error;
  Ki_output = Ki * integral_error;
  Kd_output = Kd * error_delta;

  // Add the three components to get the total output
  // Note: Check the sign of your d gain.  Check that the
  // Kd_output contribuition is the opposite of any 
  // overshoot you see using the Serial Plotter
  float total = Kp_output + Ki_output + Kd_output;

  /*
   * ===========================
   * Code below this point should not need to be changed
   * But of course, feel free to improve / experiment :)
   */

   
  //Update persistent variables.
  last_demand = demand;
  last_measurement = measurement;

  // Catching max in positive sign.
  if (total > max_output) {
    total = max_output;
  } 

  // Catching max in negative sign
  if (total < -max_output) {
    total = -max_output;
  }

  //Print debugging information if required
  if (debug) {
    Serial.print(error);
    Serial.print(",");
    Serial.print(error_delta);
    Serial.print(",");
    Serial.print(integral_error);
    Serial.print(",");
    printComponents();
  }

  //Print response if required
  if (show_response) {
    printResponse();
  }
  
  return total;
}

void PID::setMax(float newMax)
{
  if (newMax > 0) {
    max_output = newMax;
  } else {
    Serial.println("Max output must be positive");
  }
}

void PID::setDebug(bool state) {
  debug = state;
}

void PID::reset() {
  
  last_error = 0;
  integral_error = 0;
  last_millis = millis();
  
}

//This function prints measurement / demand - Good for visualiser the response on the Serial plotter
void PID::printResponse() {
  float response = last_measurement / last_demand;
  Serial.println(response);
}

void PID::setShowResponse(bool state) {
  show_response = state;
}


#endif
