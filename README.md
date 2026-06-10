# Sim-RC-Controller
 This project allows you to turn your simulator rig into a full-featured RC transmitter.  
Drive real RC vehicles using your racing wheel, flight stick, or custom joystick setup - complete with ABS braking simulation, smooth throttle curves, and head tracking support.

Sim-RC-Controller is a Python + Arduino project that bridges the virtual and physical world. It:
- Reads inputs from a racing/flight simulator using `pygame`
- Maps those inputs to CRSF-compatible channel values
- Sends those values over serial to an ELRS Module or a microcontroller (Arduino)
- ELRS Module method transmits inputs directly, while the Ardunio Method converts inputs to PPM signal for an existing TX's trainer port.

Any RC vehicle with an ELRS receiver can be used.  Cars, Trucks, Planes, Drones etc.. Be safe and have fun!

Check out the video on how to set things up: https://youtu.be/PEY1t0AU1rQ

## Setup & Installation

### Prerequisites
- Python 3.8 or higher
- [uv](https://docs.astral.sh/uv/) (modern Python package manager)
- Serial connection to Arduino or ELRS Module

### Environment Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd Sim-RC-Controller
   ```

2. **Install dependencies with uv**
   ```bash
   uv sync
   ```
   This will:
   - Create a virtual environment in `.venv/`
   - Install all required dependencies from `pyproject.toml`
   - Sync the locked versions from `uv.lock`

3. **Activate the virtual environment** (optional - uv can run scripts directly)
   ```bash
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   ```

### Running the Scripts

**Using uv directly (recommended):**
```bash
uv run python script_name.py
```

**Or with activated virtual environment:**
```bash
python script_name.py
```

### Configuration

Before running, ensure:
1. Your simulator controller (racing wheel, flight stick, etc.) is connected
2. Your Arduino/ELRS Module is connected via serial (check the COM port)
3. Update the serial port configuration in the script if needed

### Troubleshooting

- **Module not detected**: Ensure the correct serial COM port is specified
- **No input detected**: Verify your controller is detected by the OS and working in your simulator
- **Permission denied**: On Linux/Mac, you may need to add user to dialout group: `sudo usermod -a -G dialout $USER`

******************************************************************
Known issues:

Module Method:

- Slow to bind - Not entirely sure why, but it's not instant like a conventional transmitter.

- If you're experiencing orange flashing lights or "no handset", check your connections and be sure to run the python script BEFORE powering the module.



Arduino Method:

- Outputs are a little jittery, I think it's due to how the PPM frames are being generated and the memory buffer size on the pro mini/pro micro.  I wanted to keep the form factor small, but I might need to experiment with larger boards (or figure out how to get CRSF in to the trainer port)

- To avoid the jitters, use an RP2040-Zero instead of a ProMini/ProMicro.
