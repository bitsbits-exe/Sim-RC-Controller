"""
Simulator RC Controller
===USB-TTL Module Version NO HEADTRACKING===

This script interfaces with a joystick/controller and transmits control data
using the CRSF (Crossfire) protocol over serial communication. It includes
features like ABS (Anti-lock Braking System) simulation and smooth
brake deceleration for realistic vehicle control.

Features:
- Joystick input mapping to CRSF channels
- ABS simulation with configurable frequency and duty cycle
- Smooth braking deceleration
- Handbrake override functionality
- Camera pan/tilt control
- Real-time serial communication via FTDI adapter

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
# ABS and Brake Smoothing Config
# -------------------------------
BRAKE_DECEL_RATE = 3500    # Maximum deceleration rate (units per second) - controls how quickly brakes can be applied
ENABLE_ABS = True          # Toggle ABS simulation on/off
ABS_FREQUENCY = 20         # ABS pulses per second (typical automotive ABS: 15-20 Hz)
ABS_DUTY_CYCLE = 0.75      # Fraction of each pulse where brakes are applied (0.75 = 75% on, 25% off)


def apply_braking_smooth(current_throttle, brake_target, dt, now, brake_val):
    """
    Applies smooth braking with optional ABS simulation.
    
    This function simulates realistic braking behavior by:
    1. Limiting the rate of deceleration to prevent unrealistic instant stops
    2. Simulating ABS by pulsing the brake application when hard braking is detected
    
    Args:
        current_throttle (float): Current throttle/brake value (-1000 to 1000)
        brake_target (float): Target brake value to reach
        dt (float): Time delta since last update (seconds)
        now (float): Current timestamp
        brake_val (int): Raw brake pedal value (used for ABS threshold detection)
    
    Returns:
        float: New throttle value after applying braking logic
    """
    # Calculate maximum deceleration allowed this frame based on time delta
    decel_amount = BRAKE_DECEL_RATE * dt

    # ABS simulation: when brake pedal is pressed hard (>= 1400), pulse the brakes
    if ENABLE_ABS and brake_val >= 1400:
        # Calculate ABS pulse timing
        period = 1.0 / ABS_FREQUENCY  # Period of one complete ABS pulse cycle
        time_mod = now % period       # Current position within the pulse cycle
        on_time = ABS_DUTY_CYCLE * period  # Duration of brake application within each pulse
        
        # During the release phase of ABS pulse, hold current throttle (simulate brake release)
        if time_mod >= on_time:
            return current_throttle

    # Apply smooth deceleration limiting
    if current_throttle > brake_target:
        new_throttle = current_throttle - decel_amount
        # Prevent overshooting the target
        if new_throttle < brake_target:
            new_throttle = brake_target
        return new_throttle
    else:
        return brake_target


def select_serial_port():
    """
    Interactive serial port selection for CRSF communication.
    
    Scans for available serial ports and prompts the user to select
    the FTDI adapter connected to the CRSF transmitter/receiver.
    
    Returns:
        str: Device path of selected serial port
        
    Exits:
        Program exits if no ports are found
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
    Maps joystick axis values to CRSF channel values.
    
    Converts the standard joystick range (-1.0 to +1.0) to the
    CRSF protocol range (0 to 1984, with 992 as center).
    
    Args:
        val (float): Joystick axis value (-1.0 to +1.0)
        
    Returns:
        int: CRSF channel value (0 to 1984)
    """
    # CRSF uses 11-bit resolution: 0-1984 range with 992 as center
    return int((val + 1.0) * 992)


def main():
    """
    Main control loop for the racing simulator CRSF controller.
    
    This function:
    1. Initializes serial communication with CRSF device
    2. Sets up pygame for controller input
    3. Runs the main control loop that:
       - Reads controller inputs (steering, throttle, brake, handbrake)
       - Applies ABS simulation and smoothing
       - Converts to CRSF format and transmits
       - Provides debug output
    """
    # 1) Initialize CRSF serial communication
    port = select_serial_port()
    ser = serial.Serial(port, baudrate=115200, timeout=0.01)
    print(f"Opened CRSF link on {port}\n")

    # 2) Initialize Pygame and detect available controllers
    pygame.init()
    if pygame.joystick.get_count() == 0:
        print("No joysticks found.")
        sys.exit(1)

    # Enumerate and initialize all detected controllers
    joysticks = []
    print("Detected controllers:")
    for i in range(pygame.joystick.get_count()):
        js = pygame.joystick.Joystick(i)
        js.init()
        joysticks.append(js)
        print(f"  {i}: {js.get_name()}")
    
    # Let user select primary controller
    idx = int(input("Select primary controller index: "))
    joy = joysticks[idx]
    print()

    # 3) Initialize control state variables
    engine_output = 0      # Current engine/brake output value
    neutral = 1000         # Neutral point for throttle/brake calculations
    last_time = time.time()  # For delta time calculations

    print("Starting control loop. Press Ctrl-C to quit.\n")
    
    try:
        while True:
            # Process pygame events to keep controller responsive
            pygame.event.pump()

            # Calculate time delta for frame-rate independent smoothing
            now = time.time()
            dt = now - last_time
            last_time = now

            # --- STEERING INPUT ---
            # Read steering axis (typically left stick X-axis) and map to CRSF
            raw_steer = joy.get_axis(0)  # Range: -1.0 (left) to +1.0 (right)
            steer_val = map_axis_to_crsf(raw_steer)

            # --- THROTTLE AND BRAKE INPUT ---
            # Read raw throttle and brake axis values
            raw_throttle_int = int(joy.get_axis(1) * 1000)  # Axis 1: Throttle
            raw_brake_int    = int(joy.get_axis(2) * 1000)  # Axis 2: Brake
            
            # Convert to linear scale (invert axes if needed)
            throttle_lin = neutral - raw_throttle_int
            brake_lin    = neutral - raw_brake_int

            # Combine throttle and brake inputs
            # Prioritize whichever input is stronger
            if throttle_lin > brake_lin:
                combined = throttle_lin      # Positive for acceleration
            else:
                combined = -brake_lin       # Negative for braking
            combined = int(combined / 2)    # Scale down for reasonable response

            # --- ABS BRAKING LOGIC ---
            # Apply ABS simulation when braking (negative combined value)
            if combined < 0:
                engine_output = apply_braking_smooth(engine_output, combined, dt, now, brake_lin)
            else:
                # Direct throttle application (no smoothing needed for acceleration)
                engine_output = combined

            # --- HANDBRAKE INPUT ---
            # Read handbrake axis (typically right trigger or similar)
            raw_handbrake = joy.get_axis(4)
            
            if raw_handbrake > 0.5:
                # Handbrake not engaged
                handbrake_brake = 0
            else:
                # Calculate handbrake intensity based on trigger pull
                fraction = (0.5 - raw_handbrake) / 1.5
                handbrake_brake = int(-250 - 750 * fraction)  # Strong negative value for braking
            
            # Handbrake overrides normal throttle/brake when engaged
            if handbrake_brake != 0:
                engine_output = handbrake_brake

            # --- CRSF FRAME CONSTRUCTION ---
            # Convert engine output to CRSF channel value
            engine_norm = engine_output / 1000.0  # Normalize to -1.0 to +1.0
            throttle_val = map_axis_to_crsf(engine_norm)

            # Build CRSF channel array (16 channels, initialize to center position)
            channels = [992] * 16  # 992 is center position for CRSF
            channels[0] = steer_val    # Channel 0: Steering
            channels[1] = throttle_val # Channel 1: Throttle/Brake

            # Reverse channel order for physical PWM output compatibility
            tx_channels = list(reversed(channels))
            
            # Build and transmit CRSF frame
            ser.write(crsf_build_frame(
                PacketsTypes.RC_CHANNELS_PACKED,
                {"channels": tx_channels}
            ))

            # --- DEBUG OUTPUT ---
            print(f"Steer (ch0): {steer_val}, Throttle/Brake (ch1): {throttle_val}")
            print(f"Frame: {crsf_build_frame(PacketsTypes.RC_CHANNELS_PACKED, {'channels': tx_channels}).hex()}\n")

    except KeyboardInterrupt:
        print("Exiting.")
        ser.close()


if __name__ == "__main__":
    main()