//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2020-01-09 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------

#ifndef MAX6675_H_
#define MAX6675_H_

#include <SPI.h>
#include <Sensors.h>

namespace as {

template <uint8_t CS>
class MAX6675 : public Temperature {
private:

  uint16_t readCelsius() {
    SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4,MSBFIRST,SPI_MODE1));
    digitalWrite(CS,LOW);
    delayMicroseconds(1);
    uint8_t hByte = SPI.transfer(0);
    uint8_t lByte = SPI.transfer(0);
    SPI.endTransaction();
    digitalWrite(CS,HIGH);

    if (lByte & (1<<2)) {
      DPRINTLN(F("thermocouple is unconnected"));
      return 0xFFFF;
    }

    uint16_t ivalue = ( hByte << 5 | lByte >> 3 );

    return ivalue * 0.25;
  }

public:
  MAX6675 () {}

    bool init () {
      pinMode(CS,OUTPUT);
      SPI.begin();
      _present = (readCelsius() != 0xFFFF);
      return _present;
    }

    bool measure (__attribute__((unused)) bool async=false) {
      if( present() == true ) {
         uint16_t t = readCelsius();
         if (t != 0xFFFF) {
          _temperature = t * 10;
          return true;
         } else {
           return false;
         }
      }
      return false;
    }

    int16_t temperature() {return _temperature; }
};

}

#endif /* MAX6675_H_ */
