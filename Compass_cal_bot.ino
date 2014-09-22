/*
 * Drives the bot in a circle with a radius configurable by the
 * pot connected on A0.  Takes compass readings every 100ms and
 * stores the data on FRAM connected accessible via I2C.  Then
 * retrieves that data and solves the calibration equations for
 * scale and offset.  Those values are stored in EEPROM for
 * by other sketches.
 *
 * Requires the following connections
 * 
 * Sparkfun HMC5883 breakout board on I2C
 *
 * Adafruit MB85RC256V breakout on I2C
 *
 * Rover Motor Controller Conncted as follows:
 *   Rt dir on pin 2
 *   Rt PWM on pin 3
 *   Lt dir on pin 4 
 *   Lt PWM on pin 5
 * 
 * Pot wiper conncted on pin A0
 *
*/

#include "Compass_cal.h"

#include <Direction.h>
#include <Feedback.h>
#include <Message.h>
#include <Motor.h>
#include <Plant.h>
#include <State.h>

#include <Vector.h>
#include <Rover_plant.h>
#include <Rover_plant_fb.h>
#include <Rover_plant_msg.h>

#include <Wire.h>
#include <EEPROM.h>

#include <Adafruit_FRAM_I2C.h>
#include <HMC5883Llib.h>

#define DEBUG_HEADING   0
#define DEBUG_CAL       1

/*---------Global data --------------------------------------------*/
const Address eeprom_start = 0x0000;  //starting address to store cal data in EEPROM
const Address start_address = 0x0000; //starting FRAM Address for cal measurements
boolean circle_complete = false;
int starting_head = 0;
const int tolerance = 5;      // +/- value that sets band around starting head
                              // to know when the circle is complete
const int interval = 100;     // ms delay between measurements in logged data


/*---------Cal_data struct used to store calibration data---------*/
Cal_data cal_data;

/*---------Feedback Object Shared by Controller and Plant----------*/
Rover_plant_fb<Rover_plant_msg> p_fb;

/*---------Construct Rover_plant and Motors------------------------*/
Rover_plant plant(2, p_fb);       //(2 motors, feedback_object)
Motor* mtr_lt = new Motor("lt", 4, 5);  //(name, dir_pin, pwm_pin)
Motor* mtr_rt = new Motor("rt", 2, 3);

/*---------Construct Compass --------------------------------------*/
Magnetometer compass;

/*---------Construct FRAM memory ----------------------------------*/
DAZL_fram fram(start_address);  //start_address sets internal pointer
                                //to the memory location where FRAM will
                                //start writing new data

/*---------Configure pot for input --------------------------------*/
const int knob = 0;

void setup() 
{
  Serial.begin(57600);
  Wire.begin();
  delay(1500);   //wait a moment after power up before starting the cal circle
  
  static unsigned long start_cal = millis();
  
  // start the FRAM memory
  /*
  if (fram.begin()) { 
    Serial.println("Found I2C FRAM");
  } else {
    Serial.println("No I2C FRAM found ... check your connections\r\n");
    while (1);
  }
  */
  
  // start the compass  
  if (compass.begin() != 0) {
    Serial.println("\nError connecting to Magnetometer");
    while(true);
  }
  
  //attach motors to the Rover_plant
  plant.attach(mtr_lt);
  plant.attach(mtr_rt);
  
  // set the amount of gain - Use the most sensitive
  // for reading the earths magnetic field
  // 
  // MSB/Gauss   Field Range
  // 1370     +- 0.88 Ga
  // 1090     +- 1.3 Ga
  // 820      +- 1.9 Ga
  // 660      +- 2.5 Ga
  // 440      +- 4.0 Ga
  // 390      +- 4.7 Ga
  // 330      +- 5.6 Ga
  // 230      +- 8.1 Ga
  compass.setGain(HMC5833L_GAIN_1370);
  
  // we want to get a scale value between 0 and 155 to add to the base effort
  int scale = analogRead(knob);
  scale = map(scale, 0, 1023, 0, 155);
  
  // drive a circle by adjusting the control effort to the first track.
  int rt_effort = 150;
  rt_effort += scale;
  if(rt_effort > 255) rt_effort = 255;
    
  //construct the plant message
  Rover_plant_msg control(Direction::fwd, 25, Direction::fwd, rt_effort);
  Serial.print("Right effort: ");
  Serial.println(rt_effort);
  p_fb.update(control);
  
  //get the current heading
  double start_heading;
  compass.readHeadingDeg(&start_heading);
  
  //drive in a circle and log data;
  while(!circle_complete) {
    plant.take_feedback();
    plant.drive();
    
//    log_data();
       
    // if the current head is in band with the starting head then
    // finish the circle.
    double heading;
    compass.readHeadingDeg(&heading);    
    if(heading > (start_heading - tolerance) 
       && heading < (start_heading + tolerance) 
       && millis() > (start_cal + 2500) ) 
    {
       circle_complete = true; 
    }
    #if DEBUG_HEADING == 1
      Serial.print("Start Heading: ");
      Serial.print(start_heading);
      Serial.print("\tCurrent Heading: ");
      Serial.println(heading);
    #endif 
  }
  
  //solve the calibration equations with the data provided
  solve_calibration(start_address, fram.pointer(), cal_data);
  
  //store calibration data to EEPROM
  int n = EEPROM_write_anything(eeprom_start, cal_data);
  #if DEBUG_CAL == 1
    Serial.print("Bytes written to EEPROM: ");
    Serial.println(n);
    cal_data.print();
  #endif
  
}

void loop() 
{
  while(true) {
    delay(10);
  }  
}


