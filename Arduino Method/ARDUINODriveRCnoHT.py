#!/usr/bin/env python3
"""
Simulator RC Controller
===ARDUINO VERSION===

This script interfaces with a joystick/controller and transmits control data
using the CRSF (Crossfire) protocol over serial communication. It includes
features like ABS (Anti-lock Braking System) simulation and smooth
brake deceleration for realistic vehicle control.

Features:
- Real-time joystick input processing
- CRSF protocol frame generation and transmission
- ABS pulse simulation for enhanced braking control
- Smooth brake deceleration to prevent abrupt stops
- Configurable update rates and communication parameters

Dependencies:
- pygame: For joystick input handling
- pyserial: For serial communication
- crsf_parser: For CRSF protocol frame construction

Author: bitsbits
License: MIT
This script uses the following third-party libraries:
 - pygame (LGPL v2.1)
 - pyserial (BSD 3-Clause License)
 - crsf_parser (MIT License, by Alessio Morale)
"""

import argparse
import sys
import time
import pygame
import serial
import serial.tools.list_ports
from crsf_parser.payloads import PacketsTypes
from crsf_parser.handling import crsf_build_frame

# -------------------------------
# ABS and Brake-Smoothing Config
# -------------------------------
BRAKE_DECEL_RATE = 3500    # Brake deceleration rate in units per second
ENABLE_ABS       = True    # Enable/disable ABS pulsing functionality
ABS_FREQUENCY    = 20      # ABS pulse frequency in pulses per second
ABS_DUTY_CYCLE   = 0.75    # Fraction of each ABS pulse where brakes are applied

# CRSF Protocol Configuration
MID_POINT  = 992           # Center/neutral value for CRSF channels
NUM_CH     = 16            # Number of CRSF channels to transmit
BAUD_RATE  = 115200        # Serial communication baud rate
UPDATE_HZ  = 50            # Frame transmission frequency in Hz


def list_com_ports():
    """
    Enumerate all available COM/serial ports on the system.
    
    Returns:
        list: A list of device names for available serial ports
    """
    return [p.device for p in serial.tools.list_ports.comports()]


def choose_com_port(provided_port=None):
    """
    Select a COM port for serial communication.
    
    If a specific port is provided and available, it will be used.
    Otherwise, presents an interactive menu for port selection.
    
    Args:
        provided_port (str, optional): Preferred COM port name (e.g., 'COM7')
        
    Returns:
        str: Selected COM port device name
    """
    ports = list_com_ports()
    
    # Use provided port if it exists
    if provided_port in ports:
        return provided_port
    
    # Interactive port selection
    print("Available COM ports:")
    for i, p in enumerate(ports):
        print(f"  {i}: {p}")
    choice = int(input("Select COM port index: "))
    return ports[choice]


def list_joysticks():
    """
    Get a list of all available joystick/controller names.
    
    Returns:
        list: Names of all detected joystick devices
    """
    pygame.joystick.init()
    return [pygame.joystick.Joystick(i).get_name()
            for i in range(pygame.joystick.get_count())]


def choose_joystick(provided_idx=None):
    """
    Select a joystick/controller for input.
    
    If a specific index is provided and valid, it will be used.
    Otherwise, presents an interactive menu for joystick selection.
    
    Args:
        provided_idx (int, optional): Preferred joystick index (0-based)
        
    Returns:
        int: Selected joystick index
    """
    names = list_joysticks()
    
    # Use provided index if valid
    if provided_idx is not None and 0 <= provided_idx < len(names):
        return provided_idx
    
    # Interactive joystick selection
    print("Available joysticks:")
    for i, n in enumerate(names):
        print(f"  {i}: {n}")
    return int(input("Select joystick index: "))


def map_axis_to_crsf(val: float) -> int:
    """
    Convert a normalized joystick axis value (-1.0 to 1.0) to CRSF channel value.
    
    CRSF channels typically use a range of 0-1984, with 992 as the center point.
    This function maps the standard joystick range to the CRSF range.
    
    Args:
        val (float): Joystick axis value in range [-1.0, 1.0]
        
    Returns:
        int: CRSF channel value in range [0, 1984]
    """
    # Clamp input to valid range
    v = max(-1.0, min(1.0, val))
    # Map [-1.0, 1.0] to [0, 1984] with 992 as center
    return int((v + 1.0) * MID_POINT)


def apply_braking_smooth(current, target, dt, now, brake_val):
    """
    Apply smooth braking with optional ABS (Anti-lock Braking System) simulation.
    
    This function implements realistic braking behavior by:
    1. Gradually reducing brake force over time (smooth deceleration)
    2. Simulating ABS by pulsing the brake signal when heavy braking is detected
    
    Args:
        current (float): Current brake output value
        target (float): Target brake output value
        dt (float): Time delta since last update (seconds)
        now (float): Current timestamp for ABS timing
        brake_val (int): Raw brake input value for ABS threshold detection
        
    Returns:
        float: Smoothed brake output value
    """
    # Calculate maximum deceleration for this time step
    decel = BRAKE_DECEL_RATE * dt
    
    # ABS pulse logic - activated when brake input exceeds threshold
    if ENABLE_ABS and brake_val >= 1400:
        period  = 1.0 / ABS_FREQUENCY  # Time for one complete ABS cycle
        t_mod   = now % period         # Current position within ABS cycle
        on_time = ABS_DUTY_CYCLE * period  # Duration of brake application in cycle
        
        # During ABS "off" phase, maintain current brake level (simulates release)
        if t_mod >= on_time:
            return current
    
    # Smooth deceleration - gradually reduce brake force to prevent jerky stops
    if current > target:
        new = current - decel
        return max(new, target)  # Don't overshoot the target
    
    return target


def main():
    """
    Main program loop for CRSF controller interface.
    
    This function:
    1. Parses command line arguments
    2. Initializes serial communication and joystick input
    3. Runs the main control loop that reads joystick input and transmits CRSF frames
    4. Handles cleanup on exit
    """
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Send CRSF frames with ABS/brake logic")
    parser.add_argument("--port", help="COM port (e.g. COM7)")
    parser.add_argument("--joy", type=int, help="Joystick index (0-based)")
    args = parser.parse_args()

    # Initialize serial communication
    com = choose_com_port(args.port)
    print(f"Opening {com} @ {BAUD_RATE} baud, 8E2")
    ser = serial.Serial(
        com,
        baudrate=BAUD_RATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_EVEN,    # Even parity for CRSF protocol
        stopbits=serial.STOPBITS_TWO, # 2 stop bits for CRSF protocol
        timeout=0.01
    )

    # Initialize joystick subsystem
    pygame.init()
    if pygame.joystick.get_count() == 0:
        print("No joysticks found.")
        ser.close()
        sys.exit(1)
    
    jidx = choose_joystick(args.joy)
    joy = pygame.joystick.Joystick(jidx)
    joy.init()
    print(f"Using joystick [{jidx}]: {joy.get_name()}")

    # Initialize timing and control variables
    interval = 1.0 / UPDATE_HZ  # Time between frame transmissions
    engine_out = 0              # Current engine/brake output value
    neutral    = 1000           # Neutral position for throttle/brake calculations
    last_time  = time.time()    # Timestamp for delta time calculations

    try:
        # Main control loop
        while True:
            # Calculate time delta for smooth interpolation
            now = time.time()
            dt  = now - last_time
            last_time = now

            # Update joystick state
            pygame.event.pump()

            # 1) Process steering input (axis 0)
            steer_val = map_axis_to_crsf(joy.get_axis(0))

            # 2) Process throttle and brake inputs
            raw_thr = int(joy.get_axis(1) * 1000)  # Throttle axis (typically right trigger)
            raw_brk = int(joy.get_axis(2) * 1000)  # Brake axis (typically left trigger)
            
            # Convert to linear values with neutral offset
            thr_lin = neutral - raw_thr
            brk_lin = neutral - raw_brk

            # Combine throttle and brake into single control value
            # Positive values = acceleration, negative values = braking
            combined = thr_lin if thr_lin > brk_lin else -brk_lin
            combined = int(combined / 2)  # Scale down for sensitivity

            # Apply brake smoothing when braking (negative values)
            if combined < 0:
                engine_out = apply_braking_smooth(engine_out, combined, dt, now, brk_lin)
            else:
                engine_out = combined

            # 3) Process handbrake input (axis 4) - overrides normal throttle/brake
            hb = joy.get_axis(4)
            if hb <= 0.5:  # Handbrake engaged when axis value is low
                # Calculate handbrake intensity based on axis position
                frac = (0.5 - hb) / 1.5
                hb_val = int(-250 - 750 * frac)  # Strong negative value for handbrake
                if hb_val != 0:
                    engine_out = hb_val

            # Convert engine output to CRSF channel value
            throttle_val = map_axis_to_crsf(engine_out / 1000.0)

            # 4) Build CRSF channel array
            channels = [MID_POINT] * NUM_CH  # Initialize all channels to neutral
            channels[0] = steer_val          # Channel 1: Steering
            channels[1] = throttle_val       # Channel 2: Throttle/Brake
            
            # Reverse channel order for specific receiver wiring configuration
            tx = list(reversed(channels))

            # Build and transmit CRSF frame
            frame = crsf_build_frame(
                PacketsTypes.RC_CHANNELS_PACKED,  # Standard RC channel data packet
                {"channels": tx}
            )
            ser.write(frame)

            # Debug output for monitoring
            print(f"Steer={steer_val}, Throttle/Brake={throttle_val}")
            print(frame.hex())

            # Maintain consistent update rate
            elapsed = time.time() - now
            if elapsed < interval:
                time.sleep(interval - elapsed)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        # Clean up resources
        ser.close()
        print("Serial connection closed.")


if __name__ == "__main__":
    main()