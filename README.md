# Sim-RC-Controller
 This project allows you to turn your simulator rig into a full-featured RC transmitter.  
Drive real RC vehicles using your racing wheel, flight stick, or custom joystick setup—complete with ABS braking simulation, smooth throttle curves, and head tracking support.

Sim-RC-Controller is a Python + Arduino project that bridges the virtual and physical world. It:
- Reads inputs from a racing/flight simulator using `pygame`
- Maps those inputs to CRSF-compatible channel values
- Sends those values over serial to an ELRS Module or a microcontroller (Arduino)
- ELRS Module method transmits inputs directly, while the Ardunio Method converts inputs to PPM signal for and existing TX's trainer port.

Any RC vehicle with an ELRS receiver can be used.  Cars, Trucks, Planes, Drones etc.. Be safe and have fun!

Check out the video on how to set things up: www.youtube.com/@bits-bits
