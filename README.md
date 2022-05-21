Drive and steering motor and sensor controller code and PCB design for the [Dreamboat](https://github.com/OliverColeman/dreamboat) - a motorised four-poster bed designed to move in surreal and dreamlike ways.

* Connects to the Dreamboat controller (Raspberry Pi) via USB.
* Receives commands that specify the desired drive speed and angles of the wheels.
* Implements servo control for steering of the four wheels.
* Sends telemetry to the Dreamboat controller, including:
  * Wheel steering positions
  * Current draw for each drive and steering motor
  * Battery voltage
* Designed to run on a [Teensy 4.1](https://www.pjrc.com/store/teensy41.html).
* PlatformIO is used for development.
