#!/usr/bin/env python3
"""
Micro Method RC Controller
===========================

Reads joystick input from vJoy and transmits control data wirelessly
via NRF24L01 (through Arduino Pro Micro).

Supports:
- Steering (joystick X axis)
- Throttle/Brake (joystick Y/triggers)
- Real-time control at 50Hz

Dependencies:
- pygame: For joystick input
- pyserial: For serial communication with Pro Micro

Usage:
    python MicroMethodController.py [--port COM7] [--joy 0]

Author: RC Controller
License: MIT
"""

import argparse
import sys
import time
import struct
import pygame
import serial
import serial.tools.list_ports


# ============================================================================
# CONFIGURATION
# ============================================================================

UPDATE_HZ = 50          # Update frequency (50Hz = 20ms)
BAUD_RATE = 115200     # Serial baud rate for Pro Micro
NEUTRAL = 1500         # Neutral position in µs


# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def list_com_ports():
    """List all available COM ports."""
    return [p.device for p in serial.tools.list_ports.comports()]


def choose_com_port(provided_port=None):
    """
    Select a COM port for serial communication.

    Args:
        provided_port (str, optional): Preferred COM port name

    Returns:
        str: Selected COM port device name
    """
    ports = list_com_ports()

    if provided_port and provided_port in ports:
        return provided_port

    print("Available COM ports:")
    for i, p in enumerate(ports):
        print(f"  {i}: {p}")

    if not ports:
        print("No COM ports found!")
        sys.exit(1)

    choice = int(input("Select COM port index: "))
    return ports[choice]


def list_joysticks():
    """Get list of available joystick names."""
    pygame.joystick.init()
    return [pygame.joystick.Joystick(i).get_name()
            for i in range(pygame.joystick.get_count())]


def choose_joystick(provided_idx=None):
    """
    Select a joystick for input.

    Args:
        provided_idx (int, optional): Preferred joystick index

    Returns:
        int: Selected joystick index
    """
    names = list_joysticks()

    if provided_idx is not None and 0 <= provided_idx < len(names):
        return provided_idx

    print("Available joysticks:")
    for i, n in enumerate(names):
        print(f"  {i}: {n}")

    if not names:
        print("No joysticks found!")
        sys.exit(1)

    return int(input("Select joystick index: "))


def map_axis_to_control(val: float) -> int:
    """
    Convert normalized joystick axis (-1.0 to 1.0) to control value (-255 to 255).

    Args:
        val (float): Joystick axis value [-1.0, 1.0]

    Returns:
        int: Control value [-255, 255]
    """
    v = max(-1.0, min(1.0, val))
    return int(v * 255)


# ============================================================================
# MAIN PROGRAM
# ============================================================================

def main():
    """Main control loop."""

    # Parse arguments
    parser = argparse.ArgumentParser(
        description="MicroMethod RC Controller - Send joystick data to NRF24L01 via Pro Micro"
    )
    parser.add_argument("--port", help="COM port (e.g., COM7)")
    parser.add_argument("--joy", type=int, help="Joystick index (0-based)")
    args = parser.parse_args()

    # Initialize serial communication
    com_port = choose_com_port(args.port)
    print(f"\nOpening {com_port} @ {BAUD_RATE} baud...")

    try:
        ser = serial.Serial(
            com_port,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.01
        )
    except Exception as e:
        print(f"Failed to open {com_port}: {e}")
        sys.exit(1)

    # Initialize joystick
    pygame.init()
    if pygame.joystick.get_count() == 0:
        print("No joysticks found!")
        ser.close()
        sys.exit(1)

    joy_idx = choose_joystick(args.joy)
    joy = pygame.joystick.Joystick(joy_idx)
    joy.init()
    print(f"Using joystick: {joy.get_name()}\n")

    # Control loop
    interval = 1.0 / UPDATE_HZ
    last_time = time.time()

    print("Starting control loop (Ctrl+C to stop)...\n")

    try:
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            # Update joystick state
            pygame.event.pump()

            # Read steering (X axis)
            steer_raw = joy.get_axis(0)  # -1.0 to 1.0
            steer = map_axis_to_control(steer_raw)

            # Read throttle/brake
            # Assuming axis 1 = throttle, axis 2 = brake (common gamepad layout)
            throttle_raw = joy.get_axis(1) if joy.get_numaxes() > 1 else 0
            throttle = -map_axis_to_control(throttle_raw)  # Invert for intuitive control

            # Pack control data as 2 signed shorts (4 bytes total)
            data = struct.pack('<hh', steer, throttle)

            # Send to Pro Micro
            ser.write(data)

            # Debug output
            print(f"Steer: {steer:4d}  |  Throttle: {throttle:4d}  |  {data.hex()}")

            # Maintain update rate
            elapsed = time.time() - now
            if elapsed < interval:
                time.sleep(interval - elapsed)

    except KeyboardInterrupt:
        print("\n\nExiting...")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        ser.close()
        print("Serial connection closed.")


if __name__ == "__main__":
    main()
