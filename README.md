# PIC32MMXXXXGPL036_BLDC
# This is the readme file for the Prysm BLDC project using a PIC32MMXXXXGPL036 Family chip.
# The motor is a 3-phase brushless HD motor driving a polygon.
# The nominal RPM is 30Hz, though the exact frequency is a submultiple of a ~21.5kHz MEMS.
# 
# The foundation for the halless start up is taken from Microchio AN1160. The motor
# control loop is a multi-loop feedback of velocity (Fg) and phase with an outer
# loop being a phase/frequency detector at 30Hz nominal. 


