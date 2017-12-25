/*
 * DWM1000_Anchor_Tag.h
 *
 *  Created on: Feb 12, 2016
 *      Author: lieven
 */

#ifndef DWM1000_Anchor_H_
#define DWM1000_Anchor_H_

#include <DWM1000.h>
#include <DWM1000_Message.h>
#include <EventBus.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class DWM1000_Anchor : public Actor, public DWM1000 {
 public:
  uint32_t _count;
  uint32_t _interrupts;
  uint32_t _polls;
  uint32_t _finals;
  uint32_t _blinks;
  uint32_t _resps;
  uint32_t _errs;
  uint32_t _missed;
  uint32_t _txErrors;
  uint32_t _rxErrors;
  uint32_t _sys_mask;
  uint32_t _sys_status;
  uint32_t _sys_state;
  uint32_t _delta1;
  uint32_t _delta;
  uint32_t _framesMissed;
  uint32_t _framesUnknown;
  uint32_t _framesTooLong;

 private:
  uint8_t _lastSequence;
  static DWM1000_Anchor* _anchor;
  //   enum { WAIT_POLL, WAIT_FINAL } _state;
  bool interrupt_detected;
  uint32_t _frame_len;
  PollMsg _pollMsg;
  RespMsg _respMsg;
  FinalMsg _finalMsg;
  BlinkMsg _blinkMsg;
  DwmMsg _dwmMsg;
  Str _panAddress;
  Cbor _irqEvent;
  bool _hasIrqEvent;
  float _distance;
  //   uint32_t _distanceInCm;
  uint8_t _blinkSequence;
  typedef enum {
    RCV_ANY = H("RCV_ANY"),
    RCV_POLL = H("RCV_POLL"),
    RCV_FINAL = H("RCV_FINAL")
  } State;
  State _state;
  Timeout _blinkTimer;
  TaskHandle_t _isrTask;

 public:
  DWM1000_Anchor(const char* name, SPI& spi, DigitalIn& irq, DigitalOut& reset);
  virtual ~DWM1000_Anchor();
  void mode(uint32_t m);
  void setup();
  //    void resetChip();
  //    void initSpi();
  void onEvent(Cbor& msg);
  //    void enableIsr();

  void sendReply();
  void calcFinalMsg();
  int sendRespMsg();

  void loop();
  void update(uint16_t srcAddress, uint8_t sequence);
  static void rxcallback(const dwt_callback_data_t* event);
  static void txcallback(const dwt_callback_data_t* event);

  void FSM(const dwt_callback_data_t* signal);
  void onDWEvent(const dwt_callback_data_t* event);
  FrameType readMsg(const dwt_callback_data_t* signal);
  void sendBlinkMsg();
  void handleFinalMsg();
  static void interruptAwake(void*);
};

#endif /* DWM1000_Anchor_Tag_H_ */
