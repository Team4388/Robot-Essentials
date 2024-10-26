# FIRST FRC 2024 Season (Crescendo) "Kickdrum" Robot Code.
Code for our robot named "Kickdrum"

## Driving instructions

### Driver (Port 0)
**A**: Reset the gryo. 

**Left Bumper**: Gear shift down.

**Right Bumper**: Gear shift up.

### Operator (Port 1)

**Right Bumper**: Moves out the intake and starts intaking a note, when the note enters the intake move the intake back into the robot, then idles the shooter.

**Left Bumper**: Spins up the shooter to speaker speed.

**A**: *Outtakes* the *intake*. If the intake is in the robot then it will pass it off to the shooter (and its recomended that the shooter is at speaker speed when it gets handed off), if its outside the robot it will just eject it onto the floor.

**Right Trigger**: While held it moves the climbers up.

**Left Trigger**: While held it moves the climbers down.

### Programer (Port 2)

**Left Bumper**: While pressed it records using the new autonomus recording and playback system, and when released it saves the autonomus to the robot under the name that you put into shuffleboard in the "Auto Playback Name" section. (**Warning**: If the robot gets reimaged (Not redeployed) it will lose all of the auto files and you are stuck using a tool like SCP to send the autos back to the robot from this repository)

**Right Bumper**: When pressed it loads the auto from the "Auto Playback Name" in shuffleboard with the new autonomus recording and playback system, and plays it back. If the file you put in doesn't exist in autos/ in the home directory of the robot it will print the error to console but the robot will remain functional. (**Warning**: If the robot gets reimaged (Not redeployed) it will lose all of the auto files and you are stuck using a tool like SCP to send the autos back to the robot from this repository)

### Shuffleboard

**Auto Playback Name**: a string value of the auto name you wish to load or save to, this gets read at the start of the atonomus phase and loads the corisponding auto from ~/autos/ on the robot

