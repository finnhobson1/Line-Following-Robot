#ifndef _Line_follow_h
#define _Line_follow_h
#define BUZZER 6

//Number of readings to take for calibration
const int NUM_CALIBRATIONS = 100;

/* 
 *  Class to represent a single line sensor
 */
class LineSensor {
  public:

    // Required function.
    LineSensor(int pin);   //Constructor

    // Suggested functions.
    void calibrate();       //Calibrate
    int readRaw();         //Return the uncalibrated value from the sensor
    int readCalibrated();  //Return the calibrated value from the sensor

    // You may wish to add other helpful functions!
    // ...
    
  private:
  
    int pin;
    int average;
    /*
     * Add any variables needed for calibration here
     */
    
};


// Class Constructor: 
// Sets pin passed in as argument to input
LineSensor::LineSensor(int Line_pin) {
  pin = Line_pin;
  pinMode(pin, INPUT);
}

// Returns unmodified reading.
int LineSensor::readRaw() {
  return analogRead(pin);
}

// Write this function to measure any
// systematic error in your sensor and
// set some bias values.
void LineSensor::calibrate() {
  /*
   * Write code to calibrate your sensor here
   */
  long sum = 0;

  for (int i = 0; i < NUM_CALIBRATIONS; i++) {
    sum += analogRead(pin);
  }

  average = sum / NUM_CALIBRATIONS;

  analogWrite(BUZZER, 5);
  delay(100);
  analogWrite(BUZZER, 0);
  
}


// Use the above bias values to return a
// compensated ("corrected") sensor reading.
int LineSensor::readCalibrated() {
  /*
   * Write code to return a calibrated reading here
   */
   int reading = analogRead(pin) - average;
   reading = constrain(reading, 0, 1023);
   return reading;
}


#endif
