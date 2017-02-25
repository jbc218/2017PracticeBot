#This is code for FRC team 4910, East Cobb Robotics, 2017 practice robot.
This repository will likely be updated for the entire competition season. The code works with both our competition and practice bots, such that you switch a boolean in RobotMap.java.

You may also notice that we implement code that pushes a ton of data to a CSV file every iteration. We obviously plan on getting rid of that for compbot, and its only use is to provide testing data for us (using a python script to graph data). Many such functions are just for testing purposes.

The libraries needed to run this code include:

SF2 (Sensor Fusion Framework) - Direct link: http://www.kauailabs.com/public_files/sf2/sf2.zip - Chief Delphi page: https://www.chiefdelphi.com/forums/showthread.php?t=152956

NavX (AHRS 9 axis sensor) - Direct link: http://www.kauailabs.com/public_files/navx-mxp/navx-mxp.zip - Chief Delphi page: https://www.chiefdelphi.com/forums/showthread.php?t=131859 (also mentioned above)

opencsv-3.8 (CSV helper) - Sourceforge link: https://sourceforge.net/projects/opencsv/files/opencsv/3.8/

CTRLib (CANTalon libraries) - Indirect link: http://www.ctr-electronics.com/hro.html#product_tabs_technical_resources (under latest installer)

Both the SF2 and NavX libraries are automatically added to the wpilib/user/libs folder on installation. The CANTalon libraries are so large because CTR packed the Hero board libraries in with the rest of the installer.

There's also a pathfinder library but it isn't implemented, and likely won't be in the future. The SF2 library also is not implemented

There is a plotter tool to plot data from a csv file (documented in Robot.java) found in the tools folder. The syntax can be found within the file itself. If you encounter an error when running that script along the lines of "...has too many rows!", open the file and clear the last row. This is due to the robot being disabled while writing to it, which causes the data on the last row to be unfinished.