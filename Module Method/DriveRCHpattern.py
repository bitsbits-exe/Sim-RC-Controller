"""
Simulator RC Controller
== USB-TTL Module Version with H-Pattern Transmission ==

This script interfaces with racing simulation games through CRSF (Crossfire) protocol,
providing realistic H-pattern manual transmission control with per-gear acceleration,
ABS, burnout functionality, and proper clutch behavior.

Features:
- H-pattern gear shifting with clutch control
- Per-gear acceleration profiles (engine takes time to rev up)
- Clutch always forces throttle to neutral
- Stored engine speed decay during shifts
- Instant throttle cutoff when lifting foot off pedal
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

# Clutch engagement threshold — values above this indicate clutch is pressed
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

# Gear-specific data: each gear has a max_throttle and an accel_time (seconds to reach max_throttle)
# (You can tweak accel_time to taste for each gear.)
gear_data = {
    1: {'max_throttle': 166,  'accel_time': 1.0},   # 1st gear: lowest speed, highest torque
    2: {'max_throttle': 333,  'accel_time': 1.2},   # 2nd gear
    3: {'max_throttle': 500,  'accel_time': 1.4},   # 3rd gear
    4: {'max_throttle': 666,  'accel_time': 1.6},   # 4th gear
    5: {'max_throttle': 833,  'accel_time': 1.8},   # 5th gear
    6: {'max_throttle': 1000, 'accel_time': 2.0},   # 6th gear: highest speed, lowest torque
    'R': {'max_throttle': -1000, 'accel_time': 1.0}, # Reverse: full negative throttle (use accel_time=1.0s as example)
}

# Neutral position constant (serves as “no-throttle/no-brake” in axis calculations)
NEUTRAL = 1000


def apply_braking_smooth(current: int, target: int, dt: float, now: float, brake_val: int) -> int:
    """
    Apply smooth braking with optional ABS functionality.
    This function implements progressive braking that prevents wheel lockup
    by pulsing the brake pressure when ABS is active.

    Args:
        current (int): Current brake output value
        target (int): Target brake output value (negative for braking)
        dt (float): Delta time since last update (seconds)
        now (float): Current timestamp
        brake_val (int): Raw brake input value (0..2000-ish)

    Returns:
        int: Smoothed brake output value
    """
    # Calculate maximum deceleration for this frame
    decel = BRAKE_DECEL_RATE * dt

    # Apply ABS pulsing if enabled and brake pressure is high
    if ENABLE_ABS and brake_val >= 1400:  # High brake pressure threshold
        period = 1.0 / ABS_FREQUENCY  # Duration of one ABS pulse cycle
        # If in the "off" portion of the ABS duty cycle, hold current
        if (now % period) >= (ABS_DUTY_CYCLE * period):
            return current  # No change during ABS "off" time

    # Smoothly move current toward target by at most decel
    if current - decel < target:
        return target
    else:
        return current - decel


def select_serial_port() -> str:
    """
    Interactive serial port selection for CRSF communication.
    Scans available serial ports and prompts user to pick the correct FTDI port.

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
    # Convert from [-1.0, +1.0] → [0, 1984]
    return int((val + 1.0) * 992)


def main():
    """
    Main control loop for the H-pattern transmission controller.
    This function handles:
    1. Serial port initialization for CRSF communication
    2. Joystick/controller initialization
    3. Real-time input processing and gear/clutch logic
    4. Per-gear acceleration + stored engine speed
    5. ABS braking implementation
    6. Burnout mode detection and control
    7. CRSF frame transmission to simulation software
    """
    # ----------- Initialize CRSF serial communication -----------
    port = select_serial_port()
    ser = serial.Serial(port, baudrate=115200, timeout=0.01)
    print(f"Opened CRSF connection on {port}\n")

    # ----------- Initialize pygame joystick system -----------
    pygame.init()
    if pygame.joystick.get_count() == 0:
        print("No joysticks found.")
        sys.exit(1)

    # Let user pick which controller to use
    sticks = []
    print("Available controllers:")
    for i in range(pygame.joystick.get_count()):
        js = pygame.joystick.Joystick(i)
        js.init()
        sticks.append(js)
        print(f"  {i}: {js.get_name()}")

    joy = sticks[int(input("Select controller index: "))]

    # ----------- State variables for control logic -----------
    engine_speed = 0           # Current “engine output” for throttle (0 … gear_max_throttle)
    brake_output = 0           # Current brake (negative) output (for ABS + handbrake)
    prev_gear = None           # To detect gear changes and clamp engine_speed
    clutch_was_engaged = False # Previous clutch state for edge detection
    burnout_ready = False      # Whether burnout prep conditions are met
    burnout_mode = False       # Whether currently in burnout mode
    last_time = time.time()    # For delta time calculations

    print("Starting control loop; press Ctrl-C to quit.\n")

    try:
        while True:
            pygame.event.pump()
            now = time.time()
            dt = now - last_time
            last_time = now

            # ------------------- 1) Read raw controller inputs -------------------
            raw_steer  = joy.get_axis(0)                     # Steering wheel position (-1..+1)
            raw_thr    = int(joy.get_axis(1) * 1000)         # Throttle pedal (-1000..+1000)
            raw_brk    = int(joy.get_axis(2) * 1000)         # Brake pedal (-1000..+1000)
            raw_clutch = 2000 - int(joy.get_axis(3) * 1000)  # Clutch pedal inverted
            hb_axis    = joy.get_axis(4)                     # Handbrake lever (analog)

            # ------------------- 2) Combine throttle & brake inputs -------------------
            # Convert pedal positions to linear throttle/brake values around NEUTRAL=1000
            throttle_lin = NEUTRAL - raw_thr  # Positive = throttle applied
            brake_lin    = NEUTRAL - raw_brk  # Positive = brake applied

            # If throttle > brake, treat as positive forward; else negative for braking
            if throttle_lin > brake_lin:
                combined = throttle_lin      # Positive for forward throttle
            else:
                combined = -brake_lin        # Negative for braking
            combined = int(combined / 2)     # Scale down combined input

            # ------------------- 3) Handbrake override -------------------
            handbrake_brake = 0
            if hb_axis <= 0.5:  # Handbrake is pulled (analog input)
                # Map lever position to strong braking value
                frac = (0.5 - hb_axis) / 1.5
                handbrake_brake = int(-250 - 750 * frac)  # Negative output (brake)

            # ------------------- 4) Gear selection & clutch state -------------------
            clutch_engaged = (raw_clutch > CLUTCH_THRESHOLD)
            clutch_just_released = (clutch_was_engaged and not clutch_engaged)
            clutch_was_engaged = clutch_engaged

            # Which gear buttons are currently pressed?
            pressed = [g for g, b in GEAR_BUTTONS.items() if joy.get_button(b)]
            gear = pressed[0] if pressed else None  # First pressed gear, or None

            # If gear changed, clamp engine_speed to new gear's max
            if gear != prev_gear:
                if gear in gear_data and engine_speed != 0:
                    # If shifting into a forward gear, ensure engine_speed <= that gear's max_throttle
                    max_t = gear_data[gear]['max_throttle']
                    if max_t >= 0:
                        engine_speed = min(engine_speed, max_t)
                    else:
                        # Reverse gear: negative throttle
                        engine_speed = max(engine_speed, max_t)
                prev_gear = gear

            # ------------------- 5) Burnout mode logic -------------------
            # Burnout requires: 1st gear + clutch engaged + throttle pinned (raw_thr <= threshold)
            pinned = (raw_thr <= BURNOUT_PINNED_THRESHOLD)

            if not burnout_mode:
                if gear == 1 and clutch_engaged and pinned:
                    burnout_ready = True

            # Activate burnout when clutch is released after prep
            if burnout_ready and clutch_just_released:
                burnout_mode = True

            # Cancel burnout prep if conditions not met
            if not burnout_mode and not (gear == 1 and clutch_engaged and pinned):
                burnout_ready = False

            # Exit burnout mode if throttle drops too low
            if burnout_mode:
                engine_speed = combined  # During burnout, we simply mirror combined to engine_speed
                if abs(engine_speed) < BURNOUT_EXIT_FLOOR:
                    burnout_mode = False

            # ------------------- 6) Calculate “final_output” -------------------
            # We'll fill final_output either with braking or throttle, depending on state
            final_output = 0

            # 6A) Handbrake overrides everything → immediate strong negative
            if handbrake_brake != 0:
                brake_output = handbrake_brake
                final_output = brake_output
                # Do NOT touch engine_speed here

            # 6B) Pure braking (combined < 0, no handbrake)
            elif combined < 0:
                # Smooth braking via ABS
                brake_output = apply_braking_smooth(brake_output, combined, dt, now, brake_lin)
                final_output = brake_output
                # We do NOT alter engine_speed when braking

            # 6C) Burnout mode → allow full combined (no gear scaling)
            elif burnout_mode:
                final_output = engine_speed  # already set above = combined

            # 6D) Clutch pressed → force output to zero (neutral), but decay engine_speed
            elif clutch_engaged:
                # Decay engine_speed at fixed THROTTLE_DECAY_RATE units/sec
                decay_amt = THROTTLE_DECAY_RATE * dt
                if engine_speed > 0:
                    engine_speed = max(0, engine_speed - decay_amt)
                elif engine_speed < 0:
                    engine_speed = min(0, engine_speed + decay_amt)
                final_output = 0  # Always neutral when clutch is pressed

            # 6E) No clutch & gear is selected → per-gear acceleration logic
            elif gear:
                max_t = gear_data[gear]['max_throttle']
                accel_time = gear_data[gear]['accel_time']
                accel_rate = abs(max_t) / accel_time  # units per second

                # If pedal is completely off, immediate cut-off to neutral
                if combined <= 0:
                    engine_speed = 0
                    final_output = 0

                else:
                    # Determine target_throttle = (combined/1000)*max_t
                    target_throttle = int((combined / 1000.0) * max_t)

                    # If engine_speed somehow exceeds new gear's max, clamp it
                    if max_t >= 0:
                        engine_speed = min(engine_speed, max_t)
                    else:
                        engine_speed = max(engine_speed, max_t)

                    # Ramp UP if engine_speed < target_throttle
                    if max_t >= 0:
                        if engine_speed < target_throttle:
                            engine_speed = min(engine_speed + accel_rate * dt, target_throttle)
                        elif engine_speed > target_throttle:
                            # If pedal partially lifted, instantly drop to that lower target
                            engine_speed = target_throttle
                    else:
                        # Reverse gear: negative throttles
                        if engine_speed > target_throttle:
                            engine_speed = max(engine_speed - accel_rate * dt, target_throttle)
                        elif engine_speed < target_throttle:
                            engine_speed = target_throttle

                    final_output = int(engine_speed)

            # 6F) No gear selected → neutral
            else:
                engine_speed = 0
                final_output = 0

            # ------------------- 7) Transmit CRSF frame -------------------
            # Convert steering + final_output to CRSF format
            steer_ch = map_axis_to_crsf(raw_steer)            # Steering channel
            thr_ch = map_axis_to_crsf(final_output / 1000.0)  # Throttle/brake channel

            # Build a 16-channel CRSF frame (992 = center/neutral)
            channels = [992] * 16
            channels[0] = steer_ch  # Ch1: Steering
            channels[1] = thr_ch    # Ch2: Throttle/Brake

            ser.write(crsf_build_frame(
                PacketsTypes.RC_CHANNELS_PACKED,
                {"channels": list(reversed(channels))}  # CRSF expects reversed channel order
            ))

            # Debug print for monitoring (remove or comment out if too verbose)
            print(f"Gear={gear}, Clutch={'Down' if clutch_engaged else 'Up'}, "
                  f"EngineSpeed={engine_speed:.1f}, FinalOut={final_output}, Burnout={burnout_mode}")

            # Small delay to prevent pegging the CPU
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Exiting...")
        ser.close()


if __name__ == "__main__":
    main()
