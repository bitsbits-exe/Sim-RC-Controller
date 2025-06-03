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
******************************************************************
Known issues:

Module Method:

- Slow to bind - Not entirely sure why, but it's not instant like a conventional transmitter.

- If you're experiencing orange flashing lights or "no handset", check your connections and be sure to run the python script BEFORE powering the module.



Arduino Method:

- Outputs are a little jittery, I think it's due to how the PPM frames are being generated and the memory buffer size on the pro mini/pro micro.  I wanted to keep the form factor small, but I might need to experiment with larger boards (or figure out how to get CRSF in to the trainer port)
