"""
Simulator RC Controller
==USB-TTL Module Version with H-Pattern Transmission==

This script interfaces with racing simulation games through CRSF (Crossfire) protocol,
providing realistic H-pattern manual transmission control with anti-lock braking system (ABS)
and burnout functionality.

Features:
- H-pattern gear shifting with clutch control
- Progressive throttle scaling per gear
- ABS braking with pulsing effect
- Burnout mode for first gear
- Handbrake functionality
- Smooth throttle decay during gear shifts

Dependencies:
- pygame (for joystick input)
- pyserial (for CRSF communication)
- crsf_parser (for CRSF frame building)

Author: bitsbits
License: MIT
This script uses the following third-party libraries:
 - pygame (LGPL v2.1)
 - pyserial (BSD 3-Clause License)
 - crsf_parser (MIT License, by Alessio Morale)
"""

import sys
import pygame
import serial
import serial.tools.list_ports
import time
from crsf_parser.payloads import PacketsTypes
from crsf_parser.handling import crsf_build_frame

# -------------------------------
# CONFIGURATION PARAMETERS
# -------------------------------

# Clutch engagement threshold - values above this indicate clutch is pressed
CLUTCH_THRESHOLD = 1500

# Burnout detection thresholds
BURNOUT_PINNED_THRESHOLD = -900  # Raw throttle value indicating "pinned" throttle for burnout prep
BURNOUT_REMAIN_FLOOR = 300       # Minimum throttle to maintain burnout mode
BURNOUT_EXIT_FLOOR = 280         # Throttle level below which burnout mode exits

# Throttle decay rate during gear shifts (units per second)
# This creates realistic engine braking effect when clutch is engaged
THROTTLE_DECAY_RATE = 50

# ABS (Anti-lock Braking System) configuration
BRAKE_DECEL_RATE = 3000    # Maximum brake deceleration rate (units/sec)
ENABLE_ABS = True          # Enable/disable ABS functionality
ABS_FREQUENCY = 4          # ABS pulse frequency (pulses per second)
ABS_DUTY_CYCLE = 0.3       # Fraction of each pulse where brakes are applied (0.0-1.0)

# Controller button mapping for gear selection
# Maps gear positions to pygame joystick button indices
GEAR_BUTTONS = {
    1: 2,    # First gear
    2: 3,    # Second gear
    3: 4,    # Third gear
    4: 5,    # Fourth gear
    5: 6,    # Fifth gear
    6: 7,    # Sixth gear
    'R': 9   # Reverse gear
}

# Gear-specific throttle scaling configuration
# Each gear has a maximum throttle output to simulate gear ratios
gear_data = {
    1: {'max_throttle': 166},   # 1st gear: lowest speed, highest torque
    2: {'max_throttle': 333},   # 2nd gear
    3: {'max_throttle': 500},   # 3rd gear
    4: {'max_throttle': 666},   # 4th gear
    5: {'max_throttle': 833},   # 5th gear
    6: {'max_throttle': 1000},  # 6th gear: highest speed, lowest torque
    'R': {'max_throttle': -1000}, # Reverse: full negative throttle
}

# Neutral position for axis calculations (represents 0 input)
NEUTRAL = 1000


def apply_braking_smooth(current, target, dt, now, brake_val):
    """
    Apply smooth braking with optional ABS functionality.
    
    This function implements progressive braking that prevents wheel lockup
    by pulsing the brake pressure when ABS is active.
    
    Args:
        current (int): Current brake output value
        target (int): Target brake output value
        dt (float): Delta time since last update (seconds)
        now (float): Current timestamp
        brake_val (int): Raw brake input value
    
    Returns:
        int: Smoothed brake output value
    """
    # Calculate maximum deceleration for this frame
    decel = BRAKE_DECEL_RATE * dt
    
    # Apply ABS pulsing if enabled and brake pressure is high
    if ENABLE_ABS and brake_val >= 1400:  # High brake pressure threshold
        period = 1.0 / ABS_FREQUENCY  # Duration of one ABS pulse cycle
        # Check if we're in the "off" portion of the ABS duty cycle
        if (now % period) >= (ABS_DUTY_CYCLE * period):
            return current  # Don't change brake output during ABS "off" phase
    
    # Apply smooth braking transition
    return target if current - decel < target else current - decel


def select_serial_port():
    """
    Interactive serial port selection for CRSF communication.
    
    Scans available serial ports and prompts user to select the correct
    FTDI adapter port for CRSF protocol communication.
    
    Returns:
        str: Selected serial port device path
        
    Raises:
        SystemExit: If no serial ports are found
    """
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found. Is your FTDI plugged in?")
        sys.exit(1)
    
    print("Available serial ports:")
    for i, p in enumerate(ports):
        print(f"  {i}: {p.device} — {p.description}")
    
    idx = int(input("Select FTDI port index: "))
    return ports[idx].device


def map_axis_to_crsf(val: float) -> int:
    """
    Convert normalized axis value to CRSF channel format.
    
    CRSF protocol uses 11-bit channel values ranging from 0 to 1984.
    This function maps the standard joystick range (-1.0 to +1.0) to CRSF format.
    
    Args:
        val (float): Normalized axis value (-1.0 to +1.0)
    
    Returns:
        int: CRSF channel value (0 to 1984)
    """
    # Convert from [-1.0, +1.0] range to [0, 1984] CRSF range
    return int((val + 1.0) * 992)


def main():
    """
    Main control loop for the H-pattern transmission controller.
    
    This function handles:
    1. Serial port initialization for CRSF communication
    2. Joystick/controller initialization
    3. Real-time input processing and gear logic
    4. ABS braking implementation
    5. Burnout mode detection and control
    6. CRSF frame transmission to simulation software
    """
    # Initialize CRSF serial communication
    port = select_serial_port()
    ser = serial.Serial(port, baudrate=115200, timeout=0.01)
    print(f"Opened CRSF connection on {port}\n")

    # Initialize pygame joystick system
    pygame.init()
    if pygame.joystick.get_count() == 0:
        print("No joysticks found.")
        sys.exit(1)
    
    # List available controllers and let user select
    sticks = []
    print("Available controllers:")
    for i in range(pygame.joystick.get_count()):
        js = pygame.joystick.Joystick(i)
        js.init()
        sticks.append(js)
        print(f"  {i}: {js.get_name()}")
    
    joy = sticks[int(input("Select controller index: "))]

    # Initialize state variables for control logic
    engine_output = 0          # Current processed throttle/brake output
    clutch_was_engaged = False # Previous clutch state for edge detection
    burnout_ready = False      # Whether burnout preparation conditions are met
    burnout_mode = False       # Whether currently in burnout mode
    last_time = time.time()    # For delta time calculations

    print("Starting control loop; Press Ctrl-C to quit.\n")
    
    try:
        while True:
            # Update pygame event system
            pygame.event.pump()
            
            # Calculate delta time for frame-rate independent calculations
            now = time.time()
            dt = now - last_time
            last_time = now

            # 1) Read raw controller inputs
            raw_steer = joy.get_axis(0)                    # Steering wheel position
            raw_thr = int(joy.get_axis(1) * 1000)         # Throttle pedal (-1000 = full, +1000 = off)
            raw_brk = int(joy.get_axis(2) * 1000)         # Brake pedal (-1000 = full, +1000 = off)
            raw_clutch = 2000 - int(joy.get_axis(3) * 1000) # Clutch pedal (inverted)
            hb_axis = joy.get_axis(4)                      # Handbrake lever

            # 2) Combine throttle and brake inputs (mutually exclusive)
            # Convert pedal positions to linear throttle/brake values
            throttle_lin = NEUTRAL - raw_thr  # Positive = throttle applied
            brake_lin = NEUTRAL - raw_brk     # Positive = brake applied
            
            # Use whichever input is stronger (throttle or brake)
            if throttle_lin > brake_lin:
                combined = throttle_lin      # Positive for forward throttle
            else:
                combined = -brake_lin        # Negative for braking
            combined = int(combined / 2)     # Scale down the combined input

            # 3) Apply ABS smoothing to braking inputs
            if combined < 0:  # Braking
                engine_output = apply_braking_smooth(engine_output, combined, dt, now, brake_lin)
            else:  # Throttle
                engine_output = combined

            # 4) Handle handbrake override
            handbrake_brake = 0
            if hb_axis <= 0.5:  # Handbrake is pulled (analog input)
                # Calculate handbrake strength based on lever position
                frac = (0.5 - hb_axis) / 1.5
                handbrake_brake = int(-250 - 750 * frac)  # Strong negative output
            
            # Handbrake overrides all other brake/throttle inputs
            if handbrake_brake != 0:
                engine_output = handbrake_brake

            # 5) Determine gear selection and clutch state
            clutch_engaged = (raw_clutch > CLUTCH_THRESHOLD)
            clutch_just_released = (clutch_was_engaged and not clutch_engaged)
            clutch_was_engaged = clutch_engaged

            # Check which gear buttons are currently pressed
            pressed = [g for g, b in GEAR_BUTTONS.items() if joy.get_button(b)]
            gear = pressed[0] if pressed else None  # Use first pressed gear (or None)

            # 6) Burnout mode logic
            # Burnout requires: 1st gear + clutch engaged + throttle pinned
            pinned = (raw_thr <= BURNOUT_PINNED_THRESHOLD)
            
            # Enter burnout preparation mode
            if not burnout_mode:
                if gear == 1 and clutch_engaged and pinned:
                    burnout_ready = True
            
            # Activate burnout when clutch is released after preparation
            if burnout_ready and clutch_just_released:
                burnout_mode = True
            
            # Exit burnout preparation if conditions not met
            if not burnout_mode and not (gear == 1 and clutch_engaged and pinned):
                burnout_ready = False
            
            # Exit burnout mode when throttle drops too low
            if burnout_mode:
                engine_output = combined  # Use raw combined input during burnout
                if abs(engine_output) < BURNOUT_EXIT_FLOOR:
                    burnout_mode = False

            # 7) Calculate final output based on current state
            if handbrake_brake != 0:
                # Handbrake takes absolute priority
                final_output = handbrake_brake

            elif combined < 0:
                # Pure braking mode (already processed through ABS)
                final_output = engine_output

            elif burnout_mode:
                # Burnout mode: use full combined input without gear scaling
                final_output = engine_output

            elif clutch_engaged:
                # Shifting mode: gradually decay throttle for realistic engine behavior
                decay_amt = THROTTLE_DECAY_RATE * dt
                if engine_output > 0:
                    engine_output = max(0, engine_output - decay_amt)
                elif engine_output < 0:
                    engine_output = min(0, engine_output + decay_amt)
                final_output = engine_output

            elif gear:
                # Normal driving: scale throttle based on selected gear
                max_t = gear_data[gear]['max_throttle']
                scaled = int((combined / 1000) * max_t)
                
                # Clamp output to gear's maximum
                if max_t >= 0:  # Forward gears
                    final_output = min(scaled, max_t)
                else:  # Reverse gear
                    final_output = max(scaled, max_t)

            else:
                # Neutral or no gear selected: no output
                final_output = 0

            # 8) Transmit CRSF frame to simulation software
            # Convert controller inputs to CRSF channel format
            steer_ch = map_axis_to_crsf(raw_steer)                    # Steering channel
            thr_ch = map_axis_to_crsf(final_output / 1000.0)        # Throttle/brake channel

            # Initialize 16-channel CRSF frame (992 = center/neutral)
            channels = [992] * 16
            channels[0] = steer_ch  # Channel 1: Steering
            channels[1] = thr_ch    # Channel 2: Throttle/Brake

            # Build and send CRSF frame
            ser.write(crsf_build_frame(
                PacketsTypes.RC_CHANNELS_PACKED,
                {"channels": list(reversed(channels))}  # CRSF expects reversed channel order
            ))

            # Debug output for monitoring system state
            print(f"Ch0 STEER={steer_ch}, Ch1 OUT={thr_ch}, Burnout={burnout_mode}")
            
            # Small delay to prevent excessive CPU usage
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Exiting gracefully...")
        ser.close()


if __name__ == "__main__":
    main()