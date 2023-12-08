# ADArduPilot
An adaptive ArduPilot compatible with Quadcopter and Rover.

We release an autopilot that is fully compatible with all the vehicle types the ArduPilot suite was designed for (This code is only released for Quadcopter and Rover, all are based on based on version 4.1.0).
The main feature of this autopilot is that adaptation capabilities are embedded without modifying the original ArduPilot architecture, hence the name ADArduPilot. 
We test the proposed autopilot in several scenarios requiring adaptation (different payloads, different vehicle mass, environmental disturbances), 
showing that the proposed adaptation mechanism can consistently deliver improved performance  in terms of tracking error and control effort.

The flight commands in the SITL are as follows:
1. for copter: mode guided; arm throttle; takeoff 40. 
Then, the copter will take-off and  hover at the altittude 40 meters. We want copter follow an orbit, the commands are:
rc 3 1500; mode circle.

2. for rover: wp load rover3.txt; wp loop; arm throttle; mode auto.
The rover will set-off and follow a rectangle path defined in rover3.txt.
