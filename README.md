# MDPGrp7Arduino
This repo is for MDP Group 7 (Arduino). 

## Installation

### Motor Shield Library (for Arduino)
https://github.com/pololu/dual-vnh5019-motor-shield/

### Arduino PID Library
http://playground.arduino.cc/Code/PIDLibrary/

https://github.com/br3ttb/Arduino-PID-Library/

### IR Sensor Library
http://playground.arduino.cc/Main/SharpIR/

https://github.com/guillaume-rico/SharpIR/

### Running Median Library
http://playground.arduino.cc/Main/RunningMedian/

https://github.com/RobTillaart/Arduino/tree/master/libraries/RunningMedian/

### Pin Change Interrupt Library
http://playground.arduino.cc/Main/PinChangeInt/

https://code.google.com/archive/p/arduino-pinchangeint/

https://github.com/GreyGnome/PinChangeInt/

## File Strucure
- Final Code for using all the features in Final/
- Sandbox has all the trial files
- Checklist has the all the code for the clearing the checklist

## Usage
- First zip all the library files and then include in your arduino. ( note that some of the library files have been changed to implement floating point) 
- Run any of the arduino files. 
- If you are using Final/Compilation then use the serial monitor to try the following commands 
```F - Forward (1 block)
 F{1-9} - Move 1 to 9 blocks
 B - Backward (1 block)
 B{1-9} - Move 1 to 9 blocks
 L - Rotate Left 90˚
 L{0-360} - Rotate left 0-360˚
 R - Rotate Right 90˚
 R{0-360} - Rotate right 0-360˚
 X - Caliberate to front wall
 C - Caliberate to right wall
 ```
 
 ## Contribution methods
1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request :D


## History
- Added ramp function to smoothen the start and stop 
- Added buffers to help with the serial communication
- Added Auto Caliberate
- Added PID


## Credits
Jeffrey Ong
Pratyum Jagannath
