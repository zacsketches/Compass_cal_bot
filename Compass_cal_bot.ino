/*
 * Drives the bot in a circle with a radius configurable by the
 * pot connected on A2.  Takes compass readings every 100ms and
 * stores the data on FRAM connected accessible via I2C.
 *
 * Requires the following connections
 * 
 * Sparkfun HMC5883 breakout board on I2C
 *
 * Adafruit MB85RC256V breakout on I2C
 *
 * Rover Motor Controller Conncted as follows:
 *   Rt PWM on pin 3
 *   Rt dir on pin 2
 *   Lt PWM on pin 5
 *   Lt dir on pin 4 
 * 
 * 10K pot wiper conncted on pin A0
 *
*/

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

#include <Adafruit_FRAM_I2C.h>
#include <HMC5883Llib.h>

class Address_pointer {
  public:
    Address_pointer() { ptr = 0; }
    void set_pointer(const uint16_t address) { ptr = address; }
    uint16_t pointer() const {return ptr;}
    void increment() { ++ptr; }
  private:
    uint16_t ptr;   
};

Address_pointer fram_ptr;

const int compass_address = 0x1e;  //I2C 7 bit address for HMC5883

/*---------Feedback Object Shared by Controller and Plant----------*/
Rover_plant_fb<Rover_plant_msg> p_fb;

/*---------Construct Rover_plant and Motors------------------------*/
Rover_plant plant(2, p_fb);       //(2 motors, feedback_object)
Motor* mtr_lt = new Motor("lt", 4, 5);  //(name, dir_pin, pwm_pin)
Motor* mtr_rt = new Motor("rt", 2, 3);

/*---------Construct Compass --------------------------------------*/
Magnetometer compass;

/*---------Construct FRAM memory ----------------------------------*/
Adafruit_FRAM_I2C fram;

/*---------Configure pot for input --------------------------------*/
const int knob = 0;

/*---------Global data --------------------------------------------*/
boolean circle_complete = false;
int starting_head = 0;
const int tolerance = 5;      // +/- value that sets band around starting head
                              // to know when the circle is complete
const int interval = 100;     // ms delay between measurements in logged data

void setup() 
{
  Serial.begin(57600);
  Wire.begin();
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
  
  // we want to get a scale value between 0 and 155 to add to the base effort of 100
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
  int8_t ret = compass.readHeadingDeg(&start_heading);
  switch (ret)
  {
      case HMC5833L_ERROR_GAINOVERFLOW:
          Serial.println("Gain Overflow");
          return;
      case 0:
          // success
          break;
      default:
          Serial.println("Failed to read Magnetometer");
          return;
  }
  
  //drive in a circle and log data;
    while(!circle_complete) {
    plant.take_feedback();
    plant.drive();
    
//    log_data();
       
    // if the current head is in band with the starting head then
    // finish the circle.
    double heading = 370.0;
    int8_t ret = compass.readHeadingDeg(&heading);
    switch (ret)
    {
        case HMC5833L_ERROR_GAINOVERFLOW:
            Serial.println("Gain Overflow");
            break;
        case 0:
            // success
            break;
        default:
            Serial.println("Failed to read Magnetometer");
            break;
    }
    
    if(heading > (start_heading - tolerance) 
       && heading < (start_heading + tolerance) 
       && millis() > (start_cal + 2500) ) 
    {
       circle_complete = true; 
    }
    Serial.print("Start Heading: ");
    Serial.print(start_heading);
    Serial.print("\tCurrent Heading: ");
    Serial.println(heading);
  }
}

void loop() 
{
  //construct the plant message
  Rover_plant_msg control(Direction::fwd, 0, Direction::fwd, 0);
  p_fb.update(control);
  
  plant.take_feedback();
  plant.drive();
  

}

void log_data() 
{
  static unsigned long last_measurement = 0;
  static unsigned long now; 
  
  now = millis();
  
  if (now > (last_measurement + interval)) {
    //take a measurement
    int16_t x, y, z;
    int8_t ret = compass.readRaw(&x, &y, &z);
    switch (ret)
    {
        case HMC5833L_ERROR_GAINOVERFLOW:
            Serial.println("Gain Overflow");
            return;
        case 0:
            // success
            break;
        default:
            Serial.println("Failed to read raw data");
            return;
    }
      
    //log it to the FRAM
    write_16bit(x);
    write_16bit(y);
    write_16bit(z);    
    
    //update time for last measurement
    last_measurement = now;
  }
}

void write_16bit(int16_t data)
{
  static uint16_t fram_address = 0x0000;

  uint8_t hi = (data >> 8);
  uint8_t lo = (data & 0x00FF);
  
  fram.write8(fram_ptr.pointer(), hi);
  fram_ptr.increment();
  fram.write8(fram_ptr.pointer(), lo);
  fram_ptr.increment();
}
  
