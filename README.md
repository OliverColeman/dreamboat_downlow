Steering controller code and PCB design for the [Dreamboat](https://github.com/OliverColeman/dreamboat) - a motorised four-poster bed designed to move in surreal and dreamlike ways.

* Connects to the Dreamboat controller (Raspberry Pi) via USB.
* Receives commands that specify the desired angles of the wheels.
* Implements servo control of the four wheels.
* Sends telemetry including the current position of the wheels and the motor controller current draw.
* Designed to run on a [Teensy 4.1](https://www.pjrc.com/store/teensy41.html).
* PlatformIO is used for development.
