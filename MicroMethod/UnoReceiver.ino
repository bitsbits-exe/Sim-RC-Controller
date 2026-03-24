/*
 * Arduino Uno - NRF24L01 Receiver & PPM Generator
 *
 * Receives control data wirelessly from Arduino Pro Micro via NRF24L01
 * and generates PPM signal for servo/motor control
 *
 * Pin Configuration:
 * - NRF24 CE  → Pin 9
 * - NRF24 CSN → Pin 10
 * - NRF24 MOSI → Pin 11
 * - NRF24 MISO → Pin 12
 * - NRF24 SCK → Pin 13
 * - PPM Output → Pin 5 (or any PWM pin)
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
  int steer;     // -255 to 255
  int throttle;  // -255 to 255
};

// PPM Configuration
const int ppmPin = 5;
int ppmChannels[2];  // [0] = steering, [1] = throttle
unsigned long lastReceived = 0;
const unsigned long timeout = 500;  // 500ms timeout

void setup() {
  Serial.begin(9600);
  pinMode(ppmPin, OUTPUT);
  digitalWrite(ppmPin, LOW);

  // Initialize NRF24L01
  if (!radio.begin()) {
    Serial.println("NRF24 init failed!");
    while (1) delay(100);
  }

  radio.openReadingPipe(1, address);
  radio.startListening();  // Set as receiver
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);

  // Initialize PPM channels to neutral (1500µs)
  ppmChannels[0] = 1500;  // Steering neutral
  ppmChannels[1] = 1500;  // Throttle neutral

  Serial.println("Uno Receiver Ready!");
  Serial.println("Listening for NRF24 signals...");

  // Start PPM signal generation
  startPPMTimer();
}

void loop() {
  // Check for incoming NRF24 data
  if (radio.available()) {
    ControlData data;
    radio.read(&data, sizeof(data));
    lastReceived = millis();

    // Convert -255 to 255 range → 1000 to 2000 µs (servo/ESC range)
    ppmChannels[0] = map(data.steer, -255, 255, 1000, 2000);
    ppmChannels[1] = map(data.throttle, -255, 255, 1000, 2000);

    Serial.print("RX: Steer=");
    Serial.print(data.steer);
    Serial.print(" (");
    Serial.print(ppmChannels[0]);
    Serial.print("µs) Throttle=");
    Serial.print(data.throttle);
    Serial.print(" (");
    Serial.print(ppmChannels[1]);
    Serial.println("µs)");
  }

  // Safety: Return to neutral if no signal received
  if (millis() - lastReceived > timeout) {
    ppmChannels[0] = 1500;
    ppmChannels[1] = 1500;
  }

  delay(20);
}

/*
 * PPM Signal Generation
 *
 * PPM Frame Structure:
 * [PULSE_300µs][Channel1_gap][PULSE_300µs][Channel2_gap][SYNC_gap]
 *
 * Each channel: 1000-2000µs gap (total with pulse = 1300-2300µs)
 * Sync: ~5000µs to complete frame (~22.5ms total)
 */
void generatePPM() {
  for (int i = 0; i < 2; i++) {
    // Send pulse
    digitalWrite(ppmPin, HIGH);
    delayMicroseconds(300);

    // Send gap (channel value - pulse length)
    digitalWrite(ppmPin, LOW);
    delayMicroseconds(ppmChannels[i] - 300);
  }

  // Send sync pulse and gap
  digitalWrite(ppmPin, HIGH);
  delayMicroseconds(300);
  digitalWrite(ppmPin, LOW);
  delayMicroseconds(5000);  // Sync gap
}

/*
 * Timer-based PPM (more accurate than delay-based)
 * Uses Timer2 for 50Hz PPM frame rate
 */
volatile int framePhase = 0;
volatile int pulseIndex = 0;

void startPPMTimer() {
  // Configure Timer2 for PPM generation (50Hz = 20ms)
  cli();

  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  OCR2A = 78;  // ~10ms at 16MHz with prescaler 64
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22) | (1 << CS20);  // Prescaler 64
  TIMSK2 |= (1 << OCIE2A);

  sei();
}

ISR(TIMER2_COMPA_vect) {
  static unsigned int frameTime = 0;

  frameTime += 10;

  if (frameTime < 22500) {  // Frame length 22.5ms
    generatePPMTick();
  } else {
    frameTime = 0;
  }
}

void generatePPMTick() {
  // Simple PWM on pin 5
  analogWrite(ppmPin, map(ppmChannels[0], 1000, 2000, 0, 255));
}
