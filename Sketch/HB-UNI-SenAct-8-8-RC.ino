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

#define PCF8574_ADDRESS 0x20
#define RELAY_PIN_1 0
#define RELAY_PIN_2 1
#define RELAY_PIN_3 2
#define RELAY_PIN_4 3
#define RELAY_PIN_5 4
#define RELAY_PIN_6 5
#define RELAY_PIN_7 6
#define RELAY_PIN_8 7
#define RELAY_ON_STATE_INVERT false //Angepasst!


#define REMOTE_PIN_1 7
#define REMOTE_PIN_2 9
#define REMOTE_PIN_3 5
#define REMOTE_PIN_4 6
#define REMOTE_PIN_5 14
#define REMOTE_PIN_6 15
#define REMOTE_PIN_7 16
#define REMOTE_PIN_8 17

#define LED_PIN            4
#define CONFIG_BUTTON_PIN  8

// number of available peers per channel
#define PEERS_PER_SwitchChannel  3
#define PEERS_PER_RemoteChannel  3

#ifdef USE_BATTERY_MODE
#define battOp_ARGUMENT BatterySensor
#define DEV_MODEL 0x39
#else
#define battOp_ARGUMENT NoBattery
#define DEV_MODEL 0x38
#endif

#define remISR(device,chan,pin) class device##chan##ISRHandler { \
    public: \
      static void isr () { device.remoteChannel(chan).irq(); } \
  }; \
  device.remoteChannel(chan).button().init(pin); \
  if( digitalPinToInterrupt(pin) == NOT_AN_INTERRUPT ) \
    enableInterrupt(pin,device##chan##ISRHandler::isr,CHANGE); \
  else \
    attachInterrupt(digitalPinToInterrupt(pin),device##chan##ISRHandler::isr,CHANGE);

// all library classes are placed in the namespace 'as'

using namespace as;

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
  {0xf3, DEV_MODEL, 0x02},// Device ID
  "JPSENACT10",           // Device Serial
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

DEFREGISTER(Reg0, MASTERID_REGS, DREG_INTKEY)
class SwList0 : public RegList0<Reg0> {
  public:
    SwList0(uint16_t addr) : RegList0<Reg0>(addr) {}
    void defaults() {
      clear();
      intKeyVisible(true);
    }
};

DEFREGISTER(RemoteReg1, CREG_LONGPRESSTIME, CREG_AES_ACTIVE, CREG_DOUBLEPRESSTIME)
class RemoteList1 : public RegList1<RemoteReg1> {
  public:
    RemoteList1 (uint16_t addr) : RegList1<RemoteReg1>(addr) {}
    void defaults () {
      clear();
      longPressTime(1);
      // aesActive(false);
      // doublePressTime(0);
    }
};
class RemoteChannel : public Channel<Hal, RemoteList1, EmptyList, DefList4, PEERS_PER_RemoteChannel, SwList0>, public Button {
  private:
    uint8_t       repeatcnt;

  public:
    typedef Channel<Hal, RemoteList1, EmptyList, DefList4, PEERS_PER_RemoteChannel, SwList0> BaseChannel;

    RemoteChannel () : BaseChannel() {}
    virtual ~RemoteChannel () {}

    Button& button () {
      return *(Button*)this;
    }

    uint8_t status () const {
      return 0;
    }

    uint8_t flags () const {
      return 0;
    }

    virtual void state(uint8_t s) {
      DHEX(BaseChannel::number());
      Button::state(s);
      RemoteEventMsg& msg = (RemoteEventMsg&)this->device().message();
      //DPRINT("BATTERY IS LOW? "); DDECLN(this->device().battery().low());
      msg.init(this->device().nextcount(), this->number(), repeatcnt, (s == longreleased || s == longpressed), this->device().battery().low());
      if ( s == released || s == longreleased) {
        this->device().sendPeerEvent(msg, *this);
        repeatcnt++;
      }
      else if (s == longpressed) {
        this->device().broadcastPeerEvent(msg, *this);
      }
    }

    uint8_t state() const {
      return Button::state();
    }

    bool pressed () const {
      uint8_t s = state();
      return s == Button::pressed || s == Button::debounce || s == Button::longpressed;
    }
};


typedef SwitchChannel<Hal, PEERS_PER_SwitchChannel, SwList0, PCF8574Output<PCF8574_ADDRESS>>  SwChannel;

class MixDevice : public ChannelDevice<Hal, VirtBaseChannel<Hal, SwList0>, 16, SwList0> {
  public:
    VirtChannel<Hal, SwChannel, SwList0>     swChannel1,  swChannel2,  swChannel3,  swChannel4, swChannel5,  swChannel6,  swChannel7,  swChannel8;
    VirtChannel<Hal, RemoteChannel, SwList0> remChannel9, remChannel10, remChannel11, remChannel12, remChannel13, remChannel14, remChannel15, remChannel16;
  public:
    typedef ChannelDevice<Hal, VirtBaseChannel<Hal, SwList0>, 16, SwList0> DeviceType;
    MixDevice (const DeviceInfo& info, uint16_t addr) : DeviceType(info, addr) {
      DeviceType::registerChannel(swChannel1, 1);
      DeviceType::registerChannel(swChannel2, 2);
      DeviceType::registerChannel(swChannel3, 3);
      DeviceType::registerChannel(swChannel4, 4);
      DeviceType::registerChannel(swChannel5, 5);
      DeviceType::registerChannel(swChannel6, 6);
      DeviceType::registerChannel(swChannel7, 7);
      DeviceType::registerChannel(swChannel8, 8);

      DeviceType::registerChannel(remChannel9, 9);
      DeviceType::registerChannel(remChannel10, 10);
      DeviceType::registerChannel(remChannel11, 11);
      DeviceType::registerChannel(remChannel12, 12);
      DeviceType::registerChannel(remChannel13, 13);
      DeviceType::registerChannel(remChannel14, 14);
      DeviceType::registerChannel(remChannel15, 15);
      DeviceType::registerChannel(remChannel16, 16);
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
          break;
      }
    }

    RemoteChannel& remoteChannel (uint8_t num)  {
      switch (num) {
        case 9:
          return remChannel9;
          break;
        case 10:
          return remChannel10;
          break;
        case 11:
          return remChannel11;
          break;
        case 12:
          return remChannel12;
          break;
        case 13:
          return remChannel13;
          break;
        case 14:
          return remChannel14;
          break;
        case 15:
          return remChannel15;
          break;
        case 16:
          return remChannel16;
          break;
        default:
          return remChannel9;
          break;
      }
    }

    virtual void configChanged () {
    }
};
MixDevice sdev(devinfo, 0x20);
ConfigButton<MixDevice> cfgBtn(sdev);

void initPeerings (bool first) {
  // create internal peerings - CCU2 needs this
  if ( first == true ) {
    HMID devid;
    sdev.getDeviceID(devid);
    for ( uint8_t i = 1; i <= 8; ++i ) {
      Peer ipeer(devid, i + 8);
      sdev.switchChannel(i).peer(ipeer);
    }
    for ( uint8_t i = 1; i <= 8; ++i ) {
      Peer ipeer(devid, i);
      sdev.remoteChannel(i + 8).peer(ipeer);
    }
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

  remISR(sdev, 9,  REMOTE_PIN_1);
  remISR(sdev, 10, REMOTE_PIN_2);
  remISR(sdev, 11, REMOTE_PIN_3);
  remISR(sdev, 12, REMOTE_PIN_4);
  remISR(sdev, 13, REMOTE_PIN_5);
  remISR(sdev, 14, REMOTE_PIN_6);
  remISR(sdev, 15, REMOTE_PIN_7);
  remISR(sdev, 16, REMOTE_PIN_8);
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
