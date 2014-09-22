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
  
  fram.write8(fram.pointer(), hi);
  fram.increment();
  fram.write8(fram.pointer(), lo);
  fram.increment();
}

void solve_calibration(const Address start, const Address end, Cal_data& solution)
{
  Address current = start;

  Reading reading;
  Max_min_set set;
  
  while (current < end) {
    fill_reading(reading, current);    
    set.update(reading);  
    
    for(int i = 0; i < sizeof(Reading); ++i) {
      ++current;
    }   
  }
  
  fill_solution(set, solution);
  
}
  
void fill_reading(Reading& reading, const Address addr)
{
  //create local vars
  uint8_t xhi, xlo, yhi, ylo, zhi, zlo;
  Address cur = addr;
  
  //fill locals
  xhi = fram.read8(cur);
  ++cur;
  xlo = fram.read8(cur);
  ++cur;
  yhi = fram.read8(cur);
  ++cur;
  ylo = fram.read8(cur);
  ++cur;
  zhi = fram.read8(cur);
  ++cur;
  zlo = fram.read8(cur);
  
  //bit shift and fill the struct
  reading.x_raw = convert_to_signed_int(xhi, xlo);
  reading.y_raw = convert_to_signed_int(yhi, ylo);
  reading.z_raw = convert_to_signed_int(zhi, zlo);
    
}

void fill_solution(const Max_min_set& set, Cal_data& solution)
{
  /*
   * Equations for scale and offset provided by
   * Caruso, Michael, "Applications of Magnetoresistive
   * Sensors in Navigation Systems", Honeywell Application
   * Note, http://aerospace.honeywell.com/~/media/UWSAero
   * /common/documents/myaerospacecatalog-documents
   * /Defense_Brochures-documents
   * /Magnetic__Literature_Technical_Article-documents
   * /Applications_of_Magnetoresistive_Sensors_in_Navigation_Systems.pdf 
   * 
   * SF = 1 or (X|Y max- X|Y min) / (Y|X max - Y|X min)
   *    whichever is greater
   * OFF = [ (max-min) / 2 - max] * SF
   * 
  */

   solution.x_sf = (set.x_max-set.x_min) / (set.y_max - set.y_min);
   solution.x_sf = (solution.x_sf > 1) ? solution.x_sf : 1;
   solution.y_sf = (set.y_max-set.y_min) / (set.x_max - set.x_min);
   solution.y_sf = (solution.y_sf > 1) ? solution.y_sf : 1;
   
   solution.x_offset = ((set.x_max-set.x_min)/2 - set.x_max) * solution.x_sf;
   solution.y_offset = ((set.y_max-set.y_min)/2 - set.y_max) * solution.y_sf;

}

int16_t convert_to_signed_int(uint8_t MSB, uint8_t LSB)
{
  /*
   * Credit to Blaise Jarret <me@blaisejarrett.com> for the bit shifting
   * algorithm below provided in his open source library for the
   * HMC5883 magnetometer.
  */
  int16_t res = 0;
  res = (int16_t)LSB;
  res |= ((int16_t)MSB << 8);
  return res;
}



