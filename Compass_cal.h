#include <arduino.h>
#include <EEPROM.h>
#include <Adafruit_FRAM_I2C.h>

typedef uint16_t Address;

class DAZL_fram : public Adafruit_FRAM_I2C {
  public:
    DAZL_fram(Address start_address = 0) 
      :ptr(start_address) {}
    
    void set_pointer(const Address addr) { ptr = addr; }
    Address pointer() const {return ptr;}
    
    void increment() { ++ptr; }
  
  private:
    Address ptr;   
};

struct Cal_data{
  int x_offset;
  int y_offset;
  double x_sf;
  double y_sf;
  
  void print() {
    Serial.print("x_off: ");
    Serial.print(x_offset);
    Serial.print("\ty_off: ");
    Serial.print(y_offset);
    Serial.print("\tx_sf: ");
    Serial.print(x_sf);
    Serial.print("\ty_sf: ");
    Serial.println(y_sf);
  }
};

struct Reading{
  int16_t x_raw;
  int16_t y_raw;
  int16_t z_raw;
};

struct Max_min_set {
  int16_t x_max;
  int16_t x_min;
  int16_t y_max;
  int16_t y_min;
  
  Max_min_set() {
    x_max, y_max = 0;
    x_min, y_min = 32767; //max val for 16 bit int
  }
  
  void update(const Reading& data) {
    if(data.x_raw > x_max)
      x_max = data.x_raw;
    if(data.x_raw < x_min)
      x_min = data.x_raw;
    if(data.y_raw > y_max)
      y_max = data.y_raw;
    if(data.y_raw < y_min)
      y_min = data.y_raw;
  }
}; 

/*
 * EEPROM write anything is courtesy of Halley on the Arduino Forum at
 * http://forum.arduino.cc/index.php/topic,41497.0.html
*/
template <class T> int EEPROM_write_anything(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
        EEPROM.write(ee++, *p++);
    return i;
}

template <class T> int EEPROM_read_anything(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
        *p++ = EEPROM.read(ee++);
    return i;
}

