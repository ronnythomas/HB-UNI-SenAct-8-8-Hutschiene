//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2018-11-07 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// special thanks to "klassisch" from homematic-forum.de
// Angepasst 2019-01-16 auf Platine
//- -----------------------------------------------------------------------------------------------------------------------

// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <LowPower.h>
#include <actors/PCF8574.h>
#include <Switch.h>
#include <ThreeState.h>

//#define USE_BATTERY_MODE       // bei Batteriebetrieb
#define LOWBAT_VOLTAGE     22    // Batterie-Leermeldung bei Unterschreiten der Spannung von U * 10

#define PCF8574_ADDRESS 0x38
#define RELAY_PIN_1 0
#define RELAY_PIN_2 1
#define RELAY_PIN_3 2
#define RELAY_PIN_4 3
#define RELAY_PIN_5 4
#define RELAY_PIN_6 5
#define RELAY_PIN_7 6
#define RELAY_PIN_8 7
#define RELAY_ON_STATE_INVERT false //Angepasst!

#define SENS_PIN_1 7
#define SENS_PIN_2 9
#define SENS_PIN_3 5
#define SENS_PIN_4 6
#define SENS_PIN_5 14
#define SENS_PIN_6 15
#define SENS_PIN_7 16
#define SENS_PIN_8 17
#define SABOTAGE_PIN_1    3

#define LED_PIN           4
#define CONFIG_BUTTON_PIN 8

// number of available peers per channel
//#define CREATE_INTERNAL_PEERINGS
#define PEERS_PER_SwitchChannel  3
#define PEERS_PER_SENSCHANNEL    3

#ifdef USE_BATTERY_MODE
#define battOp_ARGUMENT BatterySensor
#define DEV_MODEL 0x3b
#define CYCLETIME seconds2ticks(60UL * 60 * 12 * 0.88) // 60 seconds * 60 (= minutes) * 12 (=hours) * corrective factor
#else
#define battOp_ARGUMENT NoBattery
#define DEV_MODEL 0x3a
#define CYCLETIME seconds2ticks(60UL * 3 * 0.88)  // every 3 minutes
#endif

// all library classes are placed in the namespace 'as'
using namespace as;

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
  {0xf3, DEV_MODEL, 0x01},// Device ID
  "JPSENACT01",           // Device Serial
  {0xf3, DEV_MODEL},      // Device Model
  0x10,                   // Firmware Version
  as::DeviceType::Switch, // Device Type
  {0x01, 0x00}            // Info Bytes
};

/**
   Configure the used hardware
*/
typedef AvrSPI<10, 11, 12, 13> RadioSPI;
typedef AskSin<StatusLed<LED_PIN>, battOp_ARGUMENT, Radio<RadioSPI, 2> > Hal;
Hal hal;
#ifdef USE_BATTERY_MODE
BurstDetector<Hal> bd(hal); // to wake by remote burst, taken from HM-LC-SW1-BA-PCB.ino
#endif

DEFREGISTER(Reg0, MASTERID_REGS, DREG_INTKEY, DREG_CYCLICINFOMSG, DREG_SABOTAGEMSG)
class SwList0 : public RegList0<Reg0> {
  public:
    SwList0(uint16_t addr) : RegList0<Reg0>(addr) {}
    void defaults() {
      clear();
      intKeyVisible(true);
      sabotageMsg(true);
      cycleInfoMsg(true);
    }
};


DEFREGISTER(Reg1, CREG_AES_ACTIVE, CREG_MSGFORPOS, CREG_EVENTDELAYTIME, CREG_LEDONTIME, CREG_TRANSMITTRYMAX)
class SensList1 : public RegList1<Reg1> {
  public:
    SensList1 (uint16_t addr) : RegList1<Reg1>(addr) {}
    void defaults () {
      clear();
      msgForPosA(1);
      msgForPosB(2);
      aesActive(false);
      eventDelaytime(0);
      ledOntime(100);
      transmitTryMax(6);
    }
};

typedef SwitchChannel<Hal, PEERS_PER_SwitchChannel, SwList0, PCF8574Output<PCF8574_ADDRESS>>  SwChannel;
typedef ThreeStateChannel<Hal, SwList0, SensList1, DefList4, PEERS_PER_SENSCHANNEL> SensChannel;

class MixDevice : public ChannelDevice<Hal, VirtBaseChannel<Hal, SwList0>, 16, SwList0> {
    class CycleInfoAlarm : public Alarm {
        MixDevice& dev;
      public:
        CycleInfoAlarm (MixDevice& d) : Alarm (CYCLETIME), dev(d) {}
        virtual ~CycleInfoAlarm () {}

        void trigger (AlarmClock& clock)  {
          set(CYCLETIME);
          clock.add(*this);
          dev.switchChannel(1).changed(true);
        }
    } cycle;

  public:
    VirtChannel<Hal, SwChannel, SwList0>   swChannel1,   swChannel2,   swChannel3,   swChannel4, swChannel5,   swChannel6,   swChannel7,   swChannel8;
    VirtChannel<Hal, SensChannel, SwList0> sensChannel9, sensChannel10, sensChannel11, sensChannel12, sensChannel13, sensChannel14, sensChannel15, sensChannel16;
  public:
    typedef ChannelDevice<Hal, VirtBaseChannel<Hal, SwList0>, 16, SwList0> DeviceType;
    MixDevice (const DeviceInfo& info, uint16_t addr) : DeviceType(info, addr), cycle(*this) {
      DeviceType::registerChannel(swChannel1, 1);
      DeviceType::registerChannel(swChannel2, 2);
      DeviceType::registerChannel(swChannel3, 3);
      DeviceType::registerChannel(swChannel4, 4);
      DeviceType::registerChannel(swChannel5, 5);
      DeviceType::registerChannel(swChannel6, 6);
      DeviceType::registerChannel(swChannel7, 7);
      DeviceType::registerChannel(swChannel8, 8);

      DeviceType::registerChannel(sensChannel9, 9);
      DeviceType::registerChannel(sensChannel10, 10);
      DeviceType::registerChannel(sensChannel11, 11);
      DeviceType::registerChannel(sensChannel12, 12);
      DeviceType::registerChannel(sensChannel13, 13);
      DeviceType::registerChannel(sensChannel14, 14);
      DeviceType::registerChannel(sensChannel15, 15);
      DeviceType::registerChannel(sensChannel16, 16);
    }
    virtual ~MixDevice () {}


    SwChannel& switchChannel (uint8_t num)  {
      switch (num) {
        case 1:
          return swChannel1;
          break;
        case 2:
          return swChannel2;
          break;
        case 3:
          return swChannel3;
          break;
        case 4:
          return swChannel4;
          break;
        case 5:
          return swChannel5;
          break;
        case 6:
          return swChannel6;
          break;
        case 7:
          return swChannel7;
          break;
        case 8:
          return swChannel8;
          break;
        default:
          return swChannel1;
      }
    }

    SensChannel& sensorChannel (uint8_t num)  {
      switch (num) {
        case 9:
          return sensChannel9;
          break;
        case 10:
          return sensChannel10;
          break;
        case 11:
          return sensChannel11;
          break;
        case 12:
          return sensChannel12;
          break;
        case 13:
          return sensChannel13;
          break;
        case 14:
          return sensChannel14;
          break;
        case 15:
          return sensChannel15;
          break;
        case 16:
          return sensChannel16;
          break;
        default:
          return sensChannel9;
      }
    }

    virtual void configChanged () {
      if ( /*this->getSwList0().cycleInfoMsg() ==*/ true ) {
        DPRINTLN("Activate Cycle Msg");
        sysclock.cancel(cycle);
        cycle.set(CYCLETIME);
        sysclock.add(cycle);
      }
      else {
        DPRINTLN("Deactivate Cycle Msg");
        sysclock.cancel(cycle);
      }
    }
};
MixDevice sdev(devinfo, 0x20);
ConfigButton<MixDevice> cfgBtn(sdev);

void initPeerings (bool first) {
  // create internal peerings - CCU2 needs this
  if ( first == true ) {
#ifdef CREATE_INTERNAL_PEERINGS    
    HMID devid;
    sdev.getDeviceID(devid);
    for ( uint8_t i = 1; i <= 8; ++i ) {
      Peer ipeer(devid, i + 8);
      sdev.switchChannel(i).peer(ipeer);
    }
    for ( uint8_t i = 1; i <= 8; ++i ) {
      Peer ipeer(devid, i);
      sdev.sensorChannel(i + 8).peer(ipeer);
    }
#endif    
  }
}

void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  PCF8574Output<PCF8574_ADDRESS>::init();
  bool first = sdev.init(hal);
  sdev.switchChannel(1).init(RELAY_PIN_1, RELAY_ON_STATE_INVERT);
  sdev.switchChannel(2).init(RELAY_PIN_2, RELAY_ON_STATE_INVERT);
  sdev.switchChannel(3).init(RELAY_PIN_3, RELAY_ON_STATE_INVERT);
  sdev.switchChannel(4).init(RELAY_PIN_4, RELAY_ON_STATE_INVERT);
  sdev.switchChannel(5).init(RELAY_PIN_5, RELAY_ON_STATE_INVERT);
  sdev.switchChannel(6).init(RELAY_PIN_6, RELAY_ON_STATE_INVERT);
  sdev.switchChannel(7).init(RELAY_PIN_7, RELAY_ON_STATE_INVERT);
  sdev.switchChannel(8).init(RELAY_PIN_8, RELAY_ON_STATE_INVERT);

  const uint8_t posmap[4] = {Position::State::PosA, Position::State::PosB, Position::State::PosA, Position::State::PosB};
  sdev.sensorChannel(9).init(SENS_PIN_1, SENS_PIN_1, SABOTAGE_PIN_1, posmap);
  sdev.sensorChannel(10).init(SENS_PIN_2, SENS_PIN_2, SABOTAGE_PIN_1, posmap);
  sdev.sensorChannel(11).init(SENS_PIN_3, SENS_PIN_3, SABOTAGE_PIN_1, posmap);
  sdev.sensorChannel(12).init(SENS_PIN_4, SENS_PIN_4, SABOTAGE_PIN_1, posmap);
  sdev.sensorChannel(13).init(SENS_PIN_5, SENS_PIN_5, SABOTAGE_PIN_1, posmap);
  sdev.sensorChannel(14).init(SENS_PIN_6, SENS_PIN_6, SABOTAGE_PIN_1, posmap);
  sdev.sensorChannel(15).init(SENS_PIN_7, SENS_PIN_7, SABOTAGE_PIN_1, posmap);
  sdev.sensorChannel(16).init(SENS_PIN_8, SENS_PIN_8, SABOTAGE_PIN_1, posmap);

  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);

  initPeerings(first);

#ifdef USE_BATTERY_MODE
  bd.enable(sysclock);
  hal.activity.stayAwake(seconds2ticks(15));
  hal.battery.low(LOWBAT_VOLTAGE);
  // measure battery every 12 hours
  hal.battery.init(seconds2ticks(60UL * 60 * 12 * 0.88), sysclock);
#endif

  sdev.initDone();
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false ) {
#ifdef USE_BATTERY_MODE
    hal.activity.savePower<Sleep<> >(hal);
#else
    hal.activity.savePower<Idle<> >(hal);
#endif
  }
}
