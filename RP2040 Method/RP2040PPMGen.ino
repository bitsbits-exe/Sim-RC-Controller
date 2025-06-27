/*
 * PPM Signal Generator for RP2040
 * 
 * This program generates a PPM (Pulse Position Modulation) signal commonly used
 * in RC (Radio Control) applications to control servos and other devices.
 * 
 * PPM is a method of encoding multiple analog control signals into a single
 * digital stream. Each channel's value is represented by the time interval
 * between pulses.
 * 
 * PPM signal is being generated on pin 10 - solder "tip" wire of jack to pin 10
 * and barrel of jack to GND
 *
 * Author: bitsbits
 * License: MIT
 */

#include <Arduino.h>
#include "pico/multicore.h"

// --- CRSF over USB serial parameters ---
#define BAUD_RATE        115200
#define PARITY_CFG       SERIAL_8E2
#define SYNC_BYTE        0xC8
#define TYPE_RC_PACKED   0x16
#define PAYLOAD_LEN      22
#define NUM_CHANNELS     16

// --- PPM output parameters ---
#define PPM_PIN           10       // GP10 on RP2040 Zero
#define PPM_CHANNELS      8
#define PPM_FRAME_LENGTH 22500UL   // 22.5 ms total frame
#define PPM_PULSE_LENGTH 300UL     // 300 µs separator
#define PPM_MIN_PULSE    700UL     // clamp low
#define PPM_MAX_PULSE   1700UL     // clamp high

// Parser state
enum State { WAIT_SYNC, READ_LEN, READ_TYPE, READ_PAYLOAD, READ_CRC };
static State parserState = WAIT_SYNC;

// CRSF buffers
static uint8_t  frameLen;
static uint8_t  payload[PAYLOAD_LEN];
static uint8_t  payloadIdx;
static uint16_t channels[NUM_CHANNELS];

// Double-buffer for PPM pulse widths
static uint16_t buf0[PPM_CHANNELS];
static uint16_t buf1[PPM_CHANNELS];
// pointer that core1 will read from
static volatile uint16_t *ppmValues = buf0;

// CRC-8 (poly 0xD5, initial 0) over TYPE + payload
static uint8_t crc8_update(uint8_t crc, uint8_t d) {
  crc ^= d;
  for (uint8_t i = 0; i < 8; i++)
    crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
  return crc;
}

// clamp & scale to fit in frame
static void checkFrameLength(uint16_t *vals) {
  uint32_t used = PPM_CHANNELS * PPM_PULSE_LENGTH;
  for (uint8_t i = 0; i < PPM_CHANNELS; i++) used += vals[i];
  if (used > (PPM_FRAME_LENGTH - 1000)) {
    float scale = float(PPM_FRAME_LENGTH - 2000) / used;
    for (uint8_t i = 0; i < PPM_CHANNELS; i++) {
      vals[i] = uint16_t(vals[i] * scale);
      if (vals[i] < PPM_MIN_PULSE) vals[i] = PPM_MIN_PULSE;
    }
  }
}

// Core 1: continuous PPM generation
void core1_ppm() {
  pinMode(PPM_PIN, OUTPUT);
  digitalWrite(PPM_PIN, HIGH);
  while (true) {
    uint32_t elapsed = 0;
    for (uint8_t i = 0; i < PPM_CHANNELS; i++) {
      // pulse low (inverted PPM)
      digitalWrite(PPM_PIN, LOW);
      sleep_us(ppmValues[i]);
      elapsed += ppmValues[i];
      // separator high
      digitalWrite(PPM_PIN, HIGH);
      sleep_us(PPM_PULSE_LENGTH);
      elapsed += PPM_PULSE_LENGTH;
    }
    // sync gap
    uint32_t sync = PPM_FRAME_LENGTH - elapsed;
    if (sync < 3000) sync = 3000;
    sleep_us(sync);
  }
}

void setup() {
  Serial.begin(BAUD_RATE, PARITY_CFG);

  // initialize both buffers to mid-stick (992)
  uint16_t center = map(992, 0, 1984, PPM_MIN_PULSE, PPM_MAX_PULSE);
  for (uint8_t i = 0; i < PPM_CHANNELS; i++) {
    buf0[i] = buf1[i] = center;
  }

  // launch core 1 for PPM
  multicore_launch_core1(core1_ppm);
}

void loop() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    switch (parserState) {
      case WAIT_SYNC:
        if (b == SYNC_BYTE) parserState = READ_LEN;
        break;
      case READ_LEN:
        frameLen = b;
        parserState = READ_TYPE;
        break;
      case READ_TYPE:
        if (b == TYPE_RC_PACKED) {
          payloadIdx = 0;
          parserState = READ_PAYLOAD;
        } else {
          for (uint8_t i = 0; i < frameLen + 1; i++)
            if (Serial.available()) Serial.read();
          parserState = WAIT_SYNC;
        }
        break;
      case READ_PAYLOAD:
        payload[payloadIdx++] = b;
        if (payloadIdx >= PAYLOAD_LEN) parserState = READ_CRC;
        break;
      case READ_CRC: {
        uint8_t crc = crc8_update(0, TYPE_RC_PACKED);
        for (uint8_t i = 0; i < PAYLOAD_LEN; i++)
          crc = crc8_update(crc, payload[i]);
        if (crc == b) {
          for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
            uint32_t bitPos = ch * 11;
            uint32_t byteIx = bitPos >> 3;
            uint8_t off     = bitPos & 7;
            uint32_t raw = uint32_t(payload[byteIx])
                         | (uint32_t(payload[byteIx+1])<<8)
                         | (uint32_t(payload[byteIx+2])<<16);
            channels[ch] = (raw >> off) & 0x07FF;
          }
          // build into back-buffer
          uint16_t *target = (ppmValues == buf0) ? buf1 : buf0;
          for (uint8_t i = 0; i < PPM_CHANNELS; i++) {
            target[i] = map(channels[i], 0, 1984,
                            PPM_MIN_PULSE, PPM_MAX_PULSE);
          }
          checkFrameLength(target);
          noInterrupts();
            ppmValues = target;
          interrupts();
        }
        parserState = WAIT_SYNC;
        break;
      }
    }
  }
}
