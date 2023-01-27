# Simple Pico VESC Display
A dashboard display for Personal Electric Vehicles based on the Raspberry Pi Pico (RP2040) microcontroller and SSD1306 compatible displays.


# Feature goals
* Interface to VESC-compatible motor controllers through UART or CAN bus
* Visualize PEV statistics (Speed, Battery Level, Power/Current consumption, etc..) in real-time 
* Log motor controller data to microSD card


# License
This project is licensed under the [CC BY-NC 4.0 license](https://creativecommons.org/licenses/by-nc/4.0/), except for source files covered by existing licenses: 
[VESC](https://github.com/vedderb/bldc) firmware ([GNU GPL V3+](http://www.gnu.org/licenses/)).
[Button-debouncer](https://github.com/GitJer/Some_RPI-Pico_stuff) ([MIT License](https://github.com/GitJer/Some_RPI-Pico_stuff/blob/main/LICENSE))


# Building the project
I use https://hub.docker.com/r/xingrz/rpi-pico-builder to build the project. This simplifies the process of setting up the toolchain and build environment.


If you use this method, the build command should look similar to this:

```docker run --rm -it -v /c/VSARM/sdk/pico/pico-sdk:/pico-sdk -v "/c/VSARM/repos/vesc-display:/project" xingrz/rpi-pico-builder:latest bash -c 'mkdir -p build && cd build && cmake .. && make display'```


Make sure the SDK and project folder paths are adjusted to your own directories. Run the command to create the build folder and compile the project. 

