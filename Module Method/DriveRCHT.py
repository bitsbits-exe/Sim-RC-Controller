"""
Simulator RC Controller
===USB-TTL Module Version WITH HEAD TRACKING===

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
ABS_FREQUENCY = 20         # ABS pulses per second (typical real ABS: 15-30 Hz)
ABS_DUTY_CYCLE = 0.75      # Fraction of each pulse where brakes are applied (0.75 = 75% on, 25% off)


def apply_braking_smooth(current_throttle, brake_target, dt, now, brake_val):
    """
    Applies smooth braking with optional ABS simulation.
    
    This function gradually reduces throttle to simulate realistic braking behavior.
    When ABS is enabled and hard braking is detected, it pulses the brakes on/off
    to simulate real ABS behavior that prevents wheel lockup.
    
    Args:
        current_throttle (float): Current throttle/brake value
        brake_target (float): Target brake value to reach
        dt (float): Time delta since last update (seconds)
        now (float): Current timestamp for ABS timing
        brake_val (int): Raw brake input value for ABS threshold detection
    
    Returns:
        float: New throttle value after applying braking logic
    """
    # Calculate maximum deceleration allowed this frame based on configured rate
    decel_amount = BRAKE_DECEL_RATE * dt

    # ABS simulation: when brake pedal is pressed hard (>= 1400), pulse the brakes
    if ENABLE_ABS and brake_val >= 1400:
        # Calculate ABS pulse timing
        period = 1.0 / ABS_FREQUENCY  # Time for one complete ABS cycle
        time_mod = now % period        # Where we are in the current cycle
        on_time = ABS_DUTY_CYCLE * period  # How long brakes are "on" each cycle
        
        # During the "release" phase of ABS, hold current throttle (don't brake harder)
        if time_mod >= on_time:
            return current_throttle

    # Apply smooth deceleration toward brake target
    if current_throttle > brake_target:
        new_throttle = current_throttle - decel_amount
        # Don't overshoot the target
        if new_throttle < brake_target:
            new_throttle = brake_target
        return new_throttle
    else:
        # Already at or past target
        return brake_target


def select_serial_port():
    """
    Interactive serial port selection for CRSF communication.
    
    Lists all available serial ports and prompts user to select the FTDI adapter
    connected to their RC transmitter or receiver module.
    
    Returns:
        str: Selected serial port device path
    
    Exits:
        If no serial ports are found
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
    
    Converts pygame joystick axis range (-1.0 to +1.0) to CRSF channel range (0 to 1984).
    CRSF uses 11-bit resolution with 992 as center/neutral position.
    
    Args:
        val (float): Joystick axis value (-1.0 to +1.0)
    
    Returns:
        int: CRSF channel value (0 to 1984, with 992 as center)
    """
    return int((val + 1.0) * 992)


def main():
    """
    Main control loop for the RC car controller.
    
    Handles:
    1. Serial port setup for CRSF communication
    2. Joystick initialization and selection
    3. Real-time control loop with input processing
    4. ABS simulation and smooth braking
    5. CRSF frame transmission
    """
    
    # ========================================
    # 1) CRSF Serial Port Setup
    # ========================================
    port = select_serial_port()
    ser = serial.Serial(port, baudrate=115200, timeout=0.01)
    print(f"Opened CRSF link on {port}\n")

    # ========================================
    # 2) Pygame and Joystick Initialization
    # ========================================
    pygame.init()
    if pygame.joystick.get_count() == 0:
        print("No joysticks found.")
        sys.exit(1)

    # Detect and list all connected controllers
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

    # ========================================
    # 3) Initialize Control State Variables
    # ========================================
    engine_output = 0          # Current engine/brake output value
    neutral = 1000             # Neutral position for throttle/brake calculations
    last_time = time.time()    # For calculating time deltas

    print("Starting control loop. Press Ctrl-C to quit.\n")
    
    try:
        while True:
            # Update pygame event queue to get fresh joystick data
            pygame.event.pump()

            # ========================================
            # 4) Time Management
            # ========================================
            now = time.time()
            dt = now - last_time  # Time since last frame
            last_time = now

            # ========================================
            # 5) Steering Input Processing
            # ========================================
            # Map steering wheel (usually left stick X-axis) to CRSF channel 0
            raw_steer = joy.get_axis(0)
            steer_val = map_axis_to_crsf(raw_steer)

            # ========================================
            # 6) Throttle and Brake Processing with ABS
            # ========================================
            # Read raw throttle and brake inputs (usually right stick Y-axis and triggers)
            raw_throttle_int = int(joy.get_axis(1) * 1000)  # Throttle trigger/stick
            raw_brake_int    = int(joy.get_axis(2) * 1000)  # Brake trigger/stick
            
            # Convert to linear values (neutral = 1000)
            throttle_lin = neutral - raw_throttle_int
            brake_lin    = neutral - raw_brake_int

            # Combine throttle and brake inputs (throttle takes priority)
            if throttle_lin > brake_lin:
                combined = throttle_lin      # Positive = forward throttle
            else:
                combined = -brake_lin        # Negative = braking
            combined = int(combined / 2)     # Scale down for reasonable response

            # Apply ABS smoothing when braking (negative values)
            if combined < 0:
                engine_output = apply_braking_smooth(
                    engine_output,  # Current state
                    combined,       # Target brake amount
                    dt,            # Time delta for smooth ramping
                    now,           # Current time for ABS pulses
                    brake_lin      # Raw brake value for ABS threshold
                )
            else:
                # No smoothing needed for throttle
                engine_output = combined

            # ========================================
            # 7) Handbrake Override
            # ========================================
            # Handbrake (usually shoulder button or axis) overrides normal braking
            raw_handbrake = joy.get_axis(4)
            if raw_handbrake > 0.5:
                # Handbrake not engaged
                handbrake_brake = 0
            else:
                # Calculate handbrake strength based on input
                fraction = (0.5 - raw_handbrake) / 1.5
                handbrake_brake = int(-250 - 750 * fraction)  # Strong braking force
            
            # Handbrake overrides all other throttle/brake inputs
            if handbrake_brake != 0:
                engine_output = handbrake_brake

            # ========================================
            # 8) Convert Engine Output to CRSF Format
            # ========================================
            # Map engine output (-1000 to +1000) to CRSF channel 1 (throttle/brake)
            engine_norm = engine_output / 1000.0
            throttle_val = map_axis_to_crsf(engine_norm)

            # ========================================
            # 9) Camera Control (Pan/Tilt)
            # ========================================
            # Initialize all 16 CRSF channels to center position (992)
            channels = [992] * 16
            channels[0] = steer_val    # Steering
            channels[1] = throttle_val # Throttle/Brake

            # Camera pan control (usually right stick X-axis)
            raw_pan = joy.get_axis(5)
            channels[2] = map_axis_to_crsf(raw_pan)

            # Camera tilt control (usually right stick Y-axis)
            raw_tilt = joy.get_axis(6)
            channels[3] = map_axis_to_crsf(raw_tilt)

            # ========================================
            # 10) CRSF Frame Transmission
            # ========================================
            # Reverse channel order for physical PWM output compatibility
            tx_channels = list(reversed(channels))
            
            # Build and send CRSF frame
            frame = crsf_build_frame(
                PacketsTypes.RC_CHANNELS_PACKED,
                {"channels": tx_channels}
            )
            ser.write(frame)

            # ========================================
            # 11) Debug Output
            # ========================================
            print(f"Steer (ch0): {channels[0]}, Throttle (ch1): {channels[1]}")
            print(f"Pan   (ch2): {channels[2]}, Tilt     (ch3): {channels[3]}")
            print(f"Frame: {frame.hex()}\n")

    except KeyboardInterrupt:
        print("Exiting.")
        ser.close()


if __name__ == "__main__":
    main()