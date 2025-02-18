# BME498_bioprinter
Automatic scanner cylindrical coordinate robotic arm bioprinter for burn wound treatment


## TODO: 
### Arduino side
* Generalize code to receive serial command from pc.
* Add comments and doc string
* VL53L1X integration including: reduce FOV, increased timing budget, take avg, curve smoothing, offset calibration, individual SPAD calibration, and more...
* Maybe make a different version of comb_rot with linear travel
* Feb 18: New polar to cartesian error compensation code added, still need a little bit more testing.
* Finish raster code
* Fix backlash and make end travel linearly, might be a code or hardware issue.

### PC side
* Add comments, especially for IK
* Scanning system
* Feedback system

### Hardware
* More gear reduction for the 1st arm? or half/quarter step.
* Better power delivery
* Integrate ToF (holder is almost ready, not tested yet)
* Springed pen holder
* Fix backlash issue.
