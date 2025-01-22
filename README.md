# BME498_bioprinter
Automatic scanner cylindrical coordinate robotic arm bioprinter for burn wound treatment


TODO: \
Arduino side
* Fix comb_rot, min() always return 0?
* Generalize code to receive serial command from pc.
* Add comments and doc string
* VL53L1X integration including: reduce FOV, increased timing budget, take avg, curve smoothing, offset calibration, individual SPAD calibration, and more...\
PC side
* Integrate comb_rot in IK
* Add comments, especially for IK
* Scanning system
* Feedback system \
Hardware
* More gear reduction for the 1st arm? or half/quarter step.
* Better power delivery
* Integrate ToF
* Springed pen holder
