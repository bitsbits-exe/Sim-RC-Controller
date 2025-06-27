/*
 * PPM Signal Generator for Arduino
 * 
 * This program generates a PPM (Pulse Position Modulation) signal commonly used
 * in RC (Radio Control) applications to control servos and other devices.
 * 
 * PPM is a method of encoding multiple analog control signals into a single
 * digital stream. Each channel's value is represented by the time interval
 * between pulses.
 * 
 * Author: bitsbits
 * License: MIT
 */

//////////////////////CONFIGURATION///////////////////////////////
#define chanel_number 8           // Number of channels in the PPM frame (1-8 typical for RC)
#define default_servo_value 1500  // Default servo position in microseconds (neutral/center position)
#define PPM_FrLen 22500          // Total PPM frame length in microseconds (22.5ms standard)
#define PPM_PulseLen 300         // Length of sync pulses in microseconds
#define onState 1                // Pulse polarity: 1 = positive pulses, 0 = negative pulses
#define sigPin 9                 // Arduino pin for PPM signal output (must be pin 9 for Timer1)
//////////////////////////////////////////////////////////////////

/*
 * PPM Channel Values Array
 * This array holds the servo values for each channel in the PPM signal.
 * Standard servo values range from 1000µs (full left/down) to 2000µs (full right/up)
 * with 1500µs being the neutral/center position.
 * 
 * Modify these values in your main code to control connected servos/devices.
 */
int ppm[chanel_number];

/**
 * Arduino Setup Function
 * Initializes the PPM signal generator:
 * - Sets default channel values
 * - Configures output pin
 * - Sets up Timer1 for precise PPM timing
 */
void setup(){  
  // Initialize all channels to default neutral position
  for(int i=0; i<chanel_number; i++){
    ppm[i] = default_servo_value;
  }

  // Configure PPM output pin
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  // Set pin to idle state (opposite of pulse state)
  
  // Configure Timer1 for PPM generation
  cli();                    // Disable global interrupts during timer setup
  TCCR1A = 0;              // Clear Timer1 control register A
  TCCR1B = 0;              // Clear Timer1 control register B
  
  OCR1A = 100;             // Set initial compare match value (will be updated by ISR)
  TCCR1B |= (1 << WGM12);  // Enable CTC (Clear Timer on Compare) mode
  TCCR1B |= (1 << CS11);   // Set 8x prescaler (2MHz at 16MHz clock = 0.5µs resolution)
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 compare match interrupt
  sei();                   // Re-enable global interrupts
}

/**
 * Main Loop Function
 * Contains your main program logic. In this example, it demonstrates
 * how to modify PPM channel values by creating a sweeping motion on channel 1.
 * 
 * Replace this code with your own control logic.
 */
void loop(){
  // Example: Create a sweeping motion on channel 1
  static int val = 1;  // Direction of sweep (+1 or -1)
  
  // Increment/decrement channel 1 value
  ppm[0] = ppm[0] + val;
  
  // Reverse direction at limits
  if(ppm[0] >= 2000){ val = -1; }  // At maximum, start decreasing
  if(ppm[0] <= 1000){ val = 1; }   // At minimum, start increasing
  
  delay(10);  // Small delay to control sweep speed
}

/**
 * Timer1 Compare Match Interrupt Service Routine (ISR)
 * 
 * This is the heart of the PPM signal generation. It's called automatically
 * by the Arduino's Timer1 when the compare match occurs.
 * 
 * PPM Frame Structure:
 * [PULSE][Channel1_gap][PULSE][Channel2_gap]...[PULSE][Sync_gap]
 * 
 * - Each channel is represented by the time between pulses
 * - Pulse width is constant (PPM_PulseLen)
 * - Gap duration varies based on channel value (1000-2000µs)
 * - Frame ends with a sync gap to reach total frame length
 * 
 * WARNING: Do not modify this function unless you understand PPM timing!
 */
ISR(TIMER1_COMPA_vect){
  static boolean state = true;        // Tracks current state: true = start pulse, false = end pulse
  
  TCNT1 = 0;  // Reset timer counter for next compare match
  
  if(state) {
    // START OF PULSE: Set pin high and schedule pulse end
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;  // *2 because timer resolution is 0.5µs
    state = false;
  }
  else {
    // END OF PULSE: Set pin low and calculate next pulse timing
    static byte cur_chan_numb;      // Current channel being processed (0 to chanel_number-1)
    static unsigned int calc_rest;  // Accumulator for total time used in current frame
  
    digitalWrite(sigPin, !onState);  // Set pin to idle state
    state = true;

    if(cur_chan_numb >= chanel_number){
      // ALL CHANNELS COMPLETE: Generate sync gap to complete frame
      cur_chan_numb = 0;                                    // Reset for next frame
      calc_rest = calc_rest + PPM_PulseLen;                // Add final pulse length
      OCR1A = (PPM_FrLen - calc_rest) * 2;                // Schedule sync gap
      calc_rest = 0;                                        // Reset accumulator
    }
    else{
      // PROCESS NEXT CHANNEL: Schedule gap duration based on channel value
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;    // Gap = channel_value - pulse_length
      calc_rest = calc_rest + ppm[cur_chan_numb];           // Add channel time to accumulator
      cur_chan_numb++;                                      // Move to next channel
    }     
  }
}
