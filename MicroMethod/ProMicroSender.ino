/*
 * Arduino Pro Micro - NRF24L01 Sender
 *
 * Receives control data from Python script via serial (115200 baud)
 * and transmits wirelessly to Arduino Uno via NRF24L01
 *
 * Pin Configuration:
 * - NRF24 CE  → Pin 9
 * - NRF24 CSN → Pin 10
 * - NRF24 MOSI → Pin 11
 * - NRF24 MISO → Pin 12
 * - NRF24 SCK → Pin 13
 *
 * Author: RC Controller
 * License: MIT
 */

#include <SPI.h>
#include <RF24.h>

// NRF24L01 Configuration
RF24 radio(9, 10);  // CE=9, CSN=10
const byte address[6] = "00001";

// Control data structure
struct ControlData {
  int steer;     // -255 to 255 (steering)
  int throttle;  // -255 to 255 (throttle/brake)
};

void setup() {
  Serial.begin(115200);

  // Initialize NRF24L01
  if (!radio.begin()) {
    Serial.println("NRF24 init failed!");
    while (1) delay(100);
  }

  radio.openWritingPipe(address);
  radio.stopListening();  // Set as transmitter
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);  // Slower, more reliable

  Serial.println("Pro Micro Sender Ready!");
  Serial.println("Waiting for control data...");
}

void loop() {
  // Check if Python script sent data (4 bytes for 2 ints)
  if (Serial.available() >= 4) {
    ControlData data;

    // Read steering (2 bytes)
    byte steerLow = Serial.read();
    byte steerHigh = Serial.read();
    data.steer = (steerHigh << 8) | steerLow;

    // Read throttle (2 bytes)
    byte throttleLow = Serial.read();
    byte throttleHigh = Serial.read();
    data.throttle = (throttleHigh << 8) | throttleLow;

    // Transmit via NRF24
    if (radio.write(&data, sizeof(data))) {
      Serial.print("TX: Steer=");
      Serial.print(data.steer);
      Serial.print(" Throttle=");
      Serial.println(data.throttle);
    } else {
      Serial.println("TX failed!");
    }
  }

  delay(20);
}
