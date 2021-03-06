/*
 * DWM1000_Tag.h
 *
 *  Created on: Feb 12, 2016
 *      Author: lieven
 */

#ifndef DWM1000_Tag_H_
#define DWM1000_Tag_H_
#include <DWM1000.h>
#include <DWM1000_Message.h>
#include <EventBus.h>
#include <Hardware.h>
#include <Log.h>
#include <error_handler.h>
#include <map.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ANCHOR_EXPIRE_TIME 10000
class RemoteAnchor {
 public:
  uint16_t _address;
  uint64_t _expires;
  uint8_t _sequence;
  int32_t _x;
  int32_t _y;
  uint32_t _distance;

  bool expired() { return Sys::millis() > _expires; }

  void update(uint8_t sequence) {
    _expires = Sys::millis() + ANCHOR_EXPIRE_TIME;
    if (sequence > (_sequence + 1))
      INFO(" dropped %d frames from 0x%X", sequence - _sequence - 1, _address);
    _sequence = sequence;
  }

  void update(BlinkMsg& blinkMsg) {
    uint8_t sequence = blinkMsg.sequence;
    _expires = Sys::millis() + ANCHOR_EXPIRE_TIME;
    if (sequence > (_sequence + 1))
      INFO(" dropped %d frames from 0x%X", sequence - _sequence - 1, _address);
    little_endian(_x, blinkMsg.x);
    little_endian(_y, blinkMsg.y);
    little_endian(_distance, blinkMsg.distance);
    _sequence = sequence;
  }

  RemoteAnchor(uint16_t address, uint8_t sequence) {
    _address = address;
    _expires = Sys::millis() + ANCHOR_EXPIRE_TIME;
    _sequence = sequence;
  }
};

class DWM1000_Tag : public Actor, public DWM1000 {
 public:
  Str _role;
  Str _shortAddressString;
  uint32_t _sys_mask;
  uint32_t _sys_status;
  uint32_t _sys_state;
  uint32_t _count;
  uint32_t _delta1;
  uint32_t _delta;
  uint32_t _framesMissed;
  uint32_t _framesUnknown;
  uint32_t _framesTooLong;
  uint32_t _framesUnexpected;
  uint32_t _signalUnknown;
  uint32_t _respUnknown;
  uint32_t _txErrors;
  uint32_t _rxErrors;

  static DWM1000_Tag* _tag;
  uint32_t _interrupts;
  uint32_t _pollsTxd;
  uint32_t _pollsRxd;
  bool interrupt_detected;
  uint32_t _respsTxd;
  uint32_t _respsRxd;
  uint32_t _blinksTxd;
  uint32_t _blinksRxd;
  uint32_t _finalsTxd;
  uint32_t _finalsRxd;
  uint32_t _frame_len;
  BlinkMsg _blinkMsg;
  PollMsg _pollMsg;
  RespMsg _respMsg;
  FinalMsg _finalMsg;
  DwmMsg _dwmMsg;
  Str _anchors;
  uint32_t _anchorIndex;
  uint32_t _anchorMax;
  Str _panAddress;
  uint8_t _rxdSequence;

  etl::map<uint16_t, RemoteAnchor, 10> anchors;
  // std::map<uint16_t,RemoteAnchor> anchors;

  typedef enum {
    RCV_ANY = H("RCV_ANY"),
    RCV_RESP = H("RCV_RESP"),
    RCV_FINAL = H("SND_FINAL")
  } State;
  uint16_t _currentAnchor;
  State _state;
  Timeout _pollTimer;
  TaskHandle_t _isrTask;

 public:
  DWM1000_Tag(const char* name, SPI&, DigitalIn&, DigitalOut&);
  virtual ~DWM1000_Tag();

  void mode(uint32_t m);
  void setup();
  void loop();
  //    void resetChip();
  void initSpi();
  static void my_dwt_isr(void*);
  bool isInterruptDetected();
  bool clearInterrupt();

  void onEvent(Cbor& msg);

  int sendFinalMsg();
  int sendPollMsg();
  static void rxcallback(const dwt_callback_data_t* event);
  static void txcallback(const dwt_callback_data_t* event);
  void FSM(const dwt_callback_data_t* signal);
  void onDWEvent(const dwt_callback_data_t* signal);
  FrameType readMsg(const dwt_callback_data_t* signal);
  void updateAnchors(uint16_t address, uint8_t sequence);
  void updateAnchors(BlinkMsg& blinkMsg);
  void expireAnchors();
  bool pollAnchors();
  void handleBlinkMsg();
  void handleRespMsg();
  static void interruptAwake(void*);

 private:
};

#endif /* DWM1000_Tag_H_ */
