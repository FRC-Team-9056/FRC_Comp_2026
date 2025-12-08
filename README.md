# FRC_Comp_2026
Code for The 2026 Season

# To use this repository please follow the directions below:
https://www.taniarascia.com/getting-started-with-git/

# Structure
This robot is a command based robot. What that means is that the code is set up where each subystem is a small peice of the puzzle that is called in "robotcontainer". These subsystems have commands associated with them, these are found in the commands folder.

## Robot.py
This is the part of the code that the robo rio actually runs. This is where the stages of the game are called as the game moves from mode to mode.

## Constants
This is where most changable things in the code base are found. If you swap a motor to a different controller that will be found here. If you want to speed up a motor, limit amperage, change a delay time, etc. Those variables *should* be declared here and here alone. This makes modifications easier and you don't have to go chasing variables down in other portions of the code.

This does not mean that **all** changes happen here. There are still functions and arguments that need to be changed. But, if you are setting up a number or variable that you will call again and again or need to tune, declare it here.

## Robot Container
This is where the robot is initialized, each subsystem is called and set-up, and commands are established. 
The robot container is called in robot.py to run. All the commands and subsystems are defined in here.

### Subsystems
This section is where you set up the different moving components of the robot. It allows you to later structure commands that will use some or all of these subsystems and "schedule" them so that the components can work together in harmony. For example, you could structure a command that uses both the drive base, a launcher and a camera to find a target and hit it with an object. When you trigger that command, it will "schedule it" in the robots brain. The robot will then run through that command and be able to drive the base around until it is ready to launch, launch the object and then release the control of the robot back to the joysticks without having components of the  robot working against one another. 

This functionality is incredibly useful for higher level, semi-autonomous operation. (i.e. Hit a button and the robot can position itself to do something, do something, and then be ready for the next operation all within the same press of that button.) It is also useful for the autonomous period where all of the systems will have to be working together and have the potential to do find things. You could have the robot "search" "Pick up" "position" "Do thing" and back to "search" using all those subsystems effectively. 

#### Drive
This is the drivebase subsystem. This is where you can change things related to the drive arcitecture. 

## Commands
This is where the different operations for your robot are established
