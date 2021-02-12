//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2020-01-09 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------
// ci-test=yes board=328p aes=no

// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <SPI.h>
#include <AskSinPP.h>
#include <LowPower.h>

#include <MultiChannelDevice.h>
#include <sensors/Max6675.h>

#define LED_PIN           4
#define CONFIG_BUTTON_PIN 8

#define MAX6675_SO        6
#define MAX6675_SCK       7
#define MAX6675_CS        9

#define BATT_EN_PIN       5
#define BATT_MEAS_PIN     A1

// number of available peers per channel
#define PEERS_PER_CHANNEL 6


#define MSG_INTERVAL      180
#define SYSCLOCK_FACTOR   0.88

#define BATLOW            21 // LowBat Message at 2.5V


// all library classes are placed in the namespace 'as'
using namespace as;

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
  {0xf3, 0x09, 0x00},     // Device ID
  "JPTH667500",           // Device Serial
  {0xf3, 0x09},           // Device Model Indoor
  0x10,                   // Firmware Version
  as::DeviceType::THSensor, // Device Type
  {0x01, 0x00}            // Info Bytes
};

/**
   Configure the used hardware
*/
typedef LibSPI<10> SPIType;
typedef Radio<SPIType, 2> RadioType;
typedef StatusLed<LED_PIN> LedType;
typedef AskSin<LedType, BatterySensorUni<BATT_MEAS_PIN, BATT_EN_PIN>, RadioType> Hal;
Hal hal;

class WeatherEventMsg : public Message {
  public:
    void init(uint8_t msgcnt, uint16_t temp, bool batlow) {
      uint8_t t1 = (temp >> 8) & 0x7f;
      uint8_t t2 = temp & 0xff;
      if ( batlow == true ) {
        t1 |= 0x80; // set bat low bit
      }
      Message::init(0xb, msgcnt, 0x70, BIDI | WKMEUP, t1, t2);
    }
};

class WeatherChannel : public Channel<Hal, List1, EmptyList, List4, PEERS_PER_CHANNEL, List0>, public Alarm {

    WeatherEventMsg     msg;
    bool                sensOK;
    uint16_t            millis;
    MAX6675<MAX6675_SCK, MAX6675_CS, MAX6675_SO> max6675;
    uint8_t             last_flags;
  public:
    WeatherChannel () : Channel(), Alarm(5), sensOK(true), millis(0), last_flags(0xFF) {}
    virtual ~WeatherChannel () {}

    virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
      tick = seconds2ticks(MSG_INTERVAL * SYSCLOCK_FACTOR);
      clock.add(*this);
      sensOK = max6675.measure();

      msg.init(device().nextcount(), sensOK ? max6675.temperature() : -10, false);

      device().broadcastEvent(msg);

      if (last_flags != flags()) {
        delay(400);
        last_flags = flags();
        this->changed(true);
      }

    }

    void setup(Device<Hal, List0>* dev, uint8_t number, uint16_t addr) {
      Channel::setup(dev, number, addr);
      sysclock.add(*this);
      sensOK = max6675.init();
    }

    uint8_t status () const { return 0; }

    uint8_t flags  () const { return sensOK ? 0x00 : 0x01 << 1; }
};

typedef MultiChannelDevice<Hal, WeatherChannel, 1> WeatherType;
WeatherType sdev(devinfo, 0x20);
ConfigButton<WeatherType> cfgBtn(sdev);

void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  sdev.init(hal);
  hal.initBattery(60UL * 60, BATLOW, 19);
  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
  sdev.initDone();
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false ) {
    if( hal.battery.critical() ) {
      hal.sleepForever();
    }
    hal.sleep<>();
  }
}
