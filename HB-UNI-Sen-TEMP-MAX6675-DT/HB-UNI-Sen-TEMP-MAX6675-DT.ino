//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------

// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <SPI.h>
#include <AskSinPP.h>
#include <LowPower.h>

#include <Register.h>
#include <MultiChannelDevice.h>
#include <sensors/Max6675.h>


#define LED_PIN           4
#define CONFIG_BUTTON_PIN 8

#define MAX6675_SCK       7
#define MAX6675_1_SO      6
#define MAX6675_1_CS      9
#define MAX6675_2_SO      3
#define MAX6675_2_CS      A0

#define BATT_EN_PIN       5
#define BATT_MEAS_PIN     A1
#define BATTERY_MEASURE_INTERVAL  60UL*60*12 //every 12h

// number of available peers per channel
#define PEERS_PER_CHANNEL 6

#define SYSCLOCK_FACTOR   0.88

// all library classes are placed in the namespace 'as'
using namespace as;

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
  {0xf3, 0x10, 0x00},     // Device ID
  "JPTHMXDT00",           // Device Serial
  {0xf3, 0x10},           // Device Model Indoor
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
typedef AskSin<LedType, BatterySensorUni<BATT_MEAS_PIN, BATT_EN_PIN>, RadioType> BaseHal;
class Hal : public BaseHal {
public:
  void init (const HMID& id) {
    BaseHal::init(id);
    battery.init(seconds2ticks(BATTERY_MEASURE_INTERVAL),sysclock);
  }
} hal;


DEFREGISTER(Reg0, MASTERID_REGS, DREG_LOWBATLIMIT, 0x20, 0x21)
class UList0 : public RegList0<Reg0> {
  public:
    UList0(uint16_t addr) : RegList0<Reg0>(addr) {}
    bool updIntervall (uint16_t value) const {
      return this->writeRegister(0x20, (value >> 8) & 0xff) && this->writeRegister(0x21, value & 0xff);
    }
    uint16_t updIntervall () const {
      return (this->readRegister(0x20, 0) << 8) + this->readRegister(0x21, 0);
    }

    void defaults () {
      clear();
      updIntervall(180);
      lowBatLimit(22);
    }
};

class MeasureEventMsg : public Message {
  public:
    void init(uint8_t msgcnt, int tempValues[4], bool batlow, uint8_t batval) {
      Message::init(0x18, msgcnt, 0x53, (msgcnt % 20 == 1) ? (BIDI | WKMEUP) : BCAST, batlow ? 0x80 : 0x00, 0x41);
      for (uint8_t i = 0; i < 4; i++) {
        pload[i * 3] = (tempValues[i] >> 8) & 0xff;
        pload[(i * 3) + 1] = (tempValues[i]) & 0xff;
        pload[(i * 3) + 2] = 0x42 + i;
      }
      pload[12] = batval & 0xff;
    }
};

class WeatherChannel : public Channel<Hal, List1, EmptyList, List4, PEERS_PER_CHANNEL, UList0> {
  private:
    uint8_t flgs;
  public:
    WeatherChannel () : Channel(), flgs(0) {}
    virtual ~WeatherChannel () {}

    void configChanged() {
      //DPRINTLN("Config changed List1");
    }

    uint8_t status () const {
      return 0;
    }

    void flags(uint8_t f) {
      flgs = f;
    }

    uint8_t flags () const {
      return flgs;
    }
};

class UType : public MultiChannelDevice<Hal, WeatherChannel, 4, UList0> {

    class SensorArray : public Alarm {
        UType& dev;
        MAX6675<MAX6675_SCK, MAX6675_1_CS, MAX6675_1_SO> max6675_1;
        MAX6675<MAX6675_SCK, MAX6675_2_CS, MAX6675_2_SO> max6675_2;
        bool    sens_1_OK;
        bool    sens_2_OK;
        uint8_t last_flgs;

      public:
        int16_t tempValues[4] = {0, 0, 0, 0};
        SensorArray (UType& d) : Alarm(0), dev(d), sens_1_OK(true), sens_2_OK(true), last_flgs(0xff) { }

        virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
          tick = seconds2ticks(dev.getList0().updIntervall() * SYSCLOCK_FACTOR);
          sysclock.add(*this);

          sens_1_OK = max6675_1.measure();
          sens_2_OK = max6675_2.measure();

          tempValues[0] = max6675_1.temperature();
          tempValues[1] = max6675_2.temperature();


          tempValues[2] = tempValues[0] - tempValues[1];
          tempValues[3] = tempValues[1] - tempValues[0];

          DPRINT("T:"); DDEC(tempValues[0]); DPRINT(";"); DDEC(tempValues[1]); DPRINT(";"); DDEC(tempValues[2]); DPRINT(";"); DDECLN(tempValues[3]);

          MeasureEventMsg& msg = (MeasureEventMsg&)dev.message();

          msg.init(dev.nextcount(), tempValues, dev.battery().low(), dev.battery().current());
          dev.send(msg, dev.getMasterID());

          uint8_t flgs = dev.battery().low() ? 0x80 : 0x00;

          if (sens_1_OK == false) flgs |= 0x01 << 1;
          if (sens_2_OK == false) flgs |= 0x02 << 1;

          dev.channel(1).flags(flgs);

          if (flgs != last_flgs) {
            last_flgs = flgs;
            _delay_ms(400);
            dev.channel(1).changed(true);
          }
        }

        void initSensor() {
          sens_1_OK = max6675_1.init();
          sens_2_OK = max6675_2.init();
        }

    } sensarray;

  public:
    typedef MultiChannelDevice<Hal, WeatherChannel, 4, UList0> TSDevice;
    UType(const DeviceInfo& info, uint16_t addr) : TSDevice(info, addr), sensarray(*this) {}
    virtual ~UType () {}

    virtual void configChanged () {
      TSDevice::configChanged();
      DPRINTLN("Config Changed List0");
      DPRINT("tParamSelect: "); DDECLN(this->getList0().tParamSelect());
      DPRINT("localResetDisable: "); DDECLN(this->getList0().localResetDisable());
      DPRINT("updIntervall: "); DDECLN(this->getList0().updIntervall());

      uint8_t lb = max(10, this->getList0().lowBatLimit());
      DPRINT("LOWBAT ");DDECLN(lb);
      battery().low(lb);
    }

    void init (Hal& hal) {
      TSDevice::init(hal);
      sensarray.set(seconds2ticks(2));
      sensarray.initSensor();
      sysclock.add(sensarray);
    }
};

UType sdev(devinfo, 0x20);
ConfigButton<UType> cfgBtn(sdev);

void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  sdev.init(hal);
  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
  sdev.initDone();
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false ) {
    if ( hal.battery.critical() ) {
      hal.sleepForever();
    }
    hal.sleep<>();
  }
}
