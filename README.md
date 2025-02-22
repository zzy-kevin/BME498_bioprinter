# BME498_bioprinter
Automatic scanner cylindrical coordinate robotic arm bioprinter for burn wound treatment

## Testing bioprinter setup
1. Connect the esp32 to your laptop via a usb cable.
2. Move the arms so that the first arm is pointing 90 degrees to the right and have the second arm parallel to the first arm.
3. Connect the power supply to the protoboard.
4. Run Python GUI.
5. Select the COM port from the dropdown menu, make sure any serial monitor is closed.
6. You can now send command to move the printer.
   1. move_arm 0 300 150 150 will move the end of the arm to x=0, y=300mm, with both arm being 150 mm long. 
   2. raster will take a 2D arrays and draw on all points with a 1.
   3. rotate_arm_a 20 5000 will rotate the 1st arm 20 degrees with 5000 microseconds between each step (longer means slower movement speed)


## TODO: 
### Arduino side
* Generalize code to receive serial command from pc.
* Add comments and doc string
* VL53L1X integration including: reduce FOV, increased timing budget, take avg, curve smoothing, offset calibration, individual SPAD calibration, and more...

### PC side
* Add comments for GUI
* Scanning system
* Feedback system

### Hardware
* Better power delivery
* Integrate ToF (holder is ready, not tested yet)
* Springed pen holder


### Tricks to get by setup issues
#### For Mac M1 
If you system is managed by Homebrew, your library installation attempts most like WILL NOT WORK
To get around that, create a virtual environment
* python3 -m venv venv
* source venv/bin/activate
tkinter installation
* brew install python-tk
serial installation 
* pip install pyserial

### Changelog
* Feb 18: New polar to cartesian error compensation code added, still need a bit more testing.
  * Need to finish raster code
* Feb 20: New printer variable monitoring and editing.
  * Started new communication code over serial.
  * Minor GUI change
* Feb 22: 
  * New move_arm_new command that compile a list of motion which will be sent to esp32.
  * New Arduino code that receives, combines, and execute a series of motion.
  * New quarter microstepping for all motor, might not need this for Z
  * Some GUI bug fix and lots of IK fix.
  * Errors from step quantization is still off by a bit. We might need to change to tracking steps instead of degree.
    * About +- 5 degrees for 2nd arm after raster (which has 100 movement * num_segment)