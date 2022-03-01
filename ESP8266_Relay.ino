/*************************************************************

  This is a simple demo of sending and receiving some data.
  Be sure to check out other examples!
 *************************************************************/

// Template ID, Device Name and Auth Token are provided by the Blynk.Cloud
#define BLYNK_TEMPLATE_ID "<Teplate ID>"
#define BLYNK_DEVICE_NAME "<Relay Name>"
#define BLYNK_AUTH_TOKEN "<Blynk.io Authentication>"


// Comment this out to disable prints and save space
#define BLYNK_PRINT Serial


#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

// ---------------------------------------------------------------------------------------------------------------------
// ---------------------------------------- RelayController.h ----------------------------------------------------------
#include "RelayController.h"

uint8_t const RC_COMMAND[4][2][4] = {
    {{0xA0, 0x01, 0x01, 0xA2}, {0xA0, 0x01, 0x00, 0xA1}},
    {{0xA0, 0x02, 0x01, 0xA3}, {0xA0, 0x02, 0x00, 0xA2}},
    {{0xA0, 0x03, 0x01, 0xA4}, {0xA0, 0x03, 0x00, 0xA3}},
    {{0xA0, 0x04, 0x01, 0xA5}, {0xA0, 0x04, 0x00, 0xA4}}
  };


class RelayController;
typedef void (RelayController::*RCHandler)(BlynkParam const *);

typedef struct RCData {
  RelayController *rc;
  RCHandler cb;

  RCData(RelayController *p_rc, RCHandler p_cb): rc(p_rc), cb(p_cb) {}
} RCData;



class RelayController {
 public:
  RelayController(uint8_t idx, HardwareSerial *serial, BlynkWifi *blynk, uint8_t const pins[]);

  ~RelayController();

  // Friend function
  friend void relay_timer_stop(void *rc);
  friend void relay_timer_update(void *rc);

  // Interfaces functions
  void run();
  void run_timer();
  void vpin_handler(uint8_t pin_idx, BlynkParam const *param);

  // Value manipulation functions
  void switch_relay_handler(BlynkParam const *param);
  void running_time_handler(BlynkParam const *param);
  void time_on_handler(BlynkParam const *param);
  void run_indefinitely(BlynkParam const *param);
 
 protected:
  void _enable_relay();
  void _disable_relay();
  void _start_timers(int msecs, bool update_blynk);
  void _stop_timers(bool update_blynk);
  void _update_time_on();

 private:
  static uint8_t const MIN_PIN_COUNT = 4;
  static uint32_t const MAX_RUNNING_TIME = 300;     // Seconds
  static uint32_t const OVERTIME_THRESHOLD = 3;    // Seconds

  uint8_t _relay_idx;
  uint8_t _pin_count;
  uint8_t *_pins;
  HardwareSerial *_serial;
  BlynkWifi *_blynk;
  BlynkTimer *_timer;


  RCHandler *_cbs;

  // Stream data
  int _blk_switch_relay;
  int _blk_running_time;
  int _blk_time_on;
  int _blk_run_indefinitely;

  // Status values
  int _relay_status;
  int _remaining_running_msecs;
  int _timer_start_ts;
  int _timer_countdown_id;
  int _timer_time_on_id;

  // Falogs
  bool _flag_stop;
};


// ---------------------------------------------------------------------------------------------------------------------
// ---------------------------------------- RelayController.cpp --------------------------------------------------------
#include "RelayController.cpp"

void relay_timer_stop(void *p_rc) {
  RelayController *rc = (RelayController *)p_rc;
  rc->_flag_stop = true;
}

void relay_timer_update(void *p_rc) {
  RelayController *rc = (RelayController *)p_rc;
  rc->_update_time_on();
}


RelayController::RelayController(uint8_t idx, HardwareSerial *serial, BlynkWifi *blynk, uint8_t const pins[])
  : _relay_idx(idx), _serial(serial), _blynk(blynk) {
  
  _flag_stop = false;
  _blk_switch_relay = 0;
  _blk_run_indefinitely = 0;

  _timer = new BlynkTimer();
  _pin_count = sizeof(pins) / sizeof(pins[0]);
  int cb_count = _pin_count > MIN_PIN_COUNT ? _pin_count : MIN_PIN_COUNT;
  
  // Init the Blynk value
  _blk_running_time = 1;

  // Init the pins mapping & raw cbs
  _pins = new uint8_t[_pin_count];
  _cbs = new RCHandler[cb_count];
  for (int i=0;i<cb_count;++i) {
    if (i < _pin_count)
      _pins[i] = pins[i];
    _cbs[i] = NULL;
  }

  // Init the callbacks
  _cbs[0] = &RelayController::switch_relay_handler;
  _cbs[1] = &RelayController::running_time_handler;
  _cbs[2] = &RelayController::time_on_handler;
  _cbs[3] = &RelayController::run_indefinitely;

  // Init the status
  _relay_status = 0;
  _remaining_running_msecs = 0;
  _timer_start_ts = 0;
  _timer_countdown_id = -1;
  _timer_time_on_id = -1;

  this->_stop_timers(true);
}


RelayController::~RelayController() {
  delete _timer;
  delete[] _pins;
  delete[] _cbs;
}


void RelayController::run() {
  // Check the timer
  this->run_timer();

  // For whatever reason, if the timer failed to stop the relay, stop it.
  if (_relay_status) {
    int elapsed_time = millis() - _timer_start_ts;
    int max_running_time = 1000 * (MAX_RUNNING_TIME + OVERTIME_THRESHOLD);
    if (_blk_switch_relay)
      max_running_time = _blk_running_time*1000;

    if (_flag_stop || elapsed_time > max_running_time) {
      _blk_switch_relay = 0;
      _blk_run_indefinitely = 0;
      this->_disable_relay();
      this->_stop_timers(true);

      _flag_stop = false;
    }
  }
}


void RelayController::run_timer() {
  if ((_timer_countdown_id >= 0) || (_timer_time_on_id >=0))
    this->_timer->run();
}


void RelayController::vpin_handler(uint8_t pin_idx, BlynkParam const *param) {
  for (int i=0;i<_pin_count;++i)
    if ((_pins[i] == pin_idx) && _cbs[i])
    {
      (this->*_cbs[i])(param);
      break;
    }
}


void RelayController::switch_relay_handler(BlynkParam const *param) {
  int value = param->asInt();

  // Check value of Run Indefinitely button first
  if (_blk_run_indefinitely == 1) {
    this->_blynk->virtualWrite(_pins[0], 0);
    // _blk_switch_relay = 0;
  }
  else {
    if (value == 1) {
      _blk_switch_relay = 1;
      this->_enable_relay();
      this->_start_timers(_blk_running_time*1000, false);
    }
    else if (value == 0) {
      _blk_switch_relay = 0;
      this->_disable_relay();
      this->_stop_timers(false);
    }
  }
}


void RelayController::running_time_handler(BlynkParam const *param) {
  _blk_running_time = param->asInt();
  if (_blk_running_time <= 0)
    _blk_running_time = 1;
}


void RelayController::time_on_handler(BlynkParam const *param) {
}


void RelayController::run_indefinitely(BlynkParam const *param) {
  int value = param->asInt();

  // If Switch_Relay is on, do nothing
  if (_blk_switch_relay == 1) {
    this->_blynk->virtualWrite(_pins[3], 0);
  }
  else {
    if (value == 1) {
      _blk_run_indefinitely = 1;
      this->_enable_relay();
      this->_start_timers(MAX_RUNNING_TIME*1000, false);
    }
    else if (value == 0) {
      _blk_run_indefinitely = 0;
      this->_disable_relay();
      this->_stop_timers(false);
    }
  }
}


void RelayController::_enable_relay() {
  for (int i=0;i<4;++i)
    this->_serial->write(RC_COMMAND[_relay_idx][0][i]);
  this->_relay_status = 1;
}


void RelayController::_disable_relay() {
  for (int i=0;i<4;++i)
    this->_serial->write(RC_COMMAND[_relay_idx][1][i]);
  this->_relay_status = 0;
}


void RelayController::_start_timers(int msecs, bool update_blynk) {
  _remaining_running_msecs = msecs;
  _timer_start_ts = millis();

  // Timer for stopping the relay after running_time
  _timer_countdown_id = this->_timer->setTimeout(msecs, relay_timer_stop, this);
  if (_timer_countdown_id < 0) {
    this->_serial->print("Couldn't setTimeout for countdown for relay: ");
    this->_serial->println(_relay_idx);
    return;
  }

  // Timer for updating the running time
  _timer_time_on_id = this->_timer->setInterval(1000, relay_timer_update, this);
  if (_timer_time_on_id < 0) {
    this->_serial->print("Couldn't setTimeout for timeon for relay: ");
    this->_serial->println(_relay_idx);
    if (_timer_countdown_id >= 0)
      this->_timer->deleteTimer(_timer_countdown_id);
    return;
  }

  if (update_blynk)
    this->_blynk->virtualWrite(_pins[0], 1);
}


void RelayController::_stop_timers(bool update_blynk) {
  // Remove the timers
  if (_timer_countdown_id >= 0) {
    this->_timer->deleteTimer(_timer_countdown_id);
    _timer_countdown_id = -1;
  }
  if (_timer_time_on_id >= 0) {
    this->_timer->deleteTimer(_timer_time_on_id);
    _timer_time_on_id = -1;
  }

  this->_blynk->virtualWrite(_pins[2], 0);
  if (update_blynk)
    this->_blynk->virtualWrite(_pins[0], 0);
    this->_blynk->virtualWrite(_pins[3], 0);

  _remaining_running_msecs = 0;
  _timer_start_ts = 0;
}


void RelayController::_update_time_on() {
  int elapsed_msecs = millis() - _timer_start_ts;
  _remaining_running_msecs -= elapsed_msecs;

  this->_blynk->virtualWrite(_pins[2], elapsed_msecs/1000);
}

// ---------------------------------------------------------------------------------------------------------------------


char auth[] = BLYNK_AUTH_TOKEN;

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "<Local Wifi SSID>";
char pass[] = "<Local Wifi password>";


// Init the relay
uint8_t relay_pins[4][4] = {{0, 1, 2, 3}, {10, 11, 12, 13}, {20, 21, 22, 23}, {30, 31, 32, 33}};
RelayController relays[] = {
  RelayController(0, &Serial, &Blynk, relay_pins[0]), 
  RelayController(1, &Serial, &Blynk, relay_pins[1]),
  RelayController(2, &Serial, &Blynk, relay_pins[2]),
  RelayController(3, &Serial, &Blynk, relay_pins[3])
};
BlynkTimer timer;

// -----------------------------------------------------------------


// Current only support 4 relays so ignore all pins < 40
BLYNK_WRITE_DEFAULT() {
  uint8_t pin = (uint8_t)request.pin;
  if (pin < 40) {
    relays[pin/10].vpin_handler(pin, &param);
  }
}


// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
  // Change Web Link Button message to "Congratulations!"
//  Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
//  Blynk.setProperty(V3, "onImageUrl",  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
//  Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
  //Blynk.syncAll();  <=== Doesn't work, FUCK.
  for (int i=0;i<4;++i)
    for (int j=0;j<4;++j)
      Blynk.syncVirtual(relay_pins[i][j]);
}

int start_ts;
void uptimeEvent() {
  Blynk.virtualWrite(V40, (millis() - start_ts)/1000);
}


void setup()
{
  // Debug console
  Serial.begin(115200);

  Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);

  // Setup a function to be called every second
  start_ts = millis();
  timer.setInterval(1000L, uptimeEvent);
}

void loop()
{
  Blynk.run();
  timer.run();
  // You can inject your own code or combine it with other sketches.
  // Check other examples on how to communicate with Blynk. Remember
  // to avoid delay() function!
  for (int i=0;i < 4;++i)
    relays[i].run();
}
