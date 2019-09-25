# master-thesis-autonomous-parking
codes of the parallel-parking scenario.

Note that all of the files should be in the same directory.
Here is the structure of files:
1. autoParking.py: includes the main implementation 
2. maneuver.py: contains calculation for vehicle maneuver which is imported to autoParking and is called during maneuver step iby parking_maneuver() function in the main file(autoParking)
3.config.py: all of the thresholds and global values for vehicle's parameters(like steering and celocity) are difined here. This file is imported to maneuver.py file and its values are called by calculationn function.
4. parking.m: this is a matlab file which calls detector and examine the results of the detection to find parking vacancy. The output is overlap-ratio which returned to python(auto-parking)
5. interface.py: is an API to interface matlab and python.

# Instruction
In order to run python scripts, Carla-server should be executed at first, then python scripts could be called and connect to the server on port 2000 or 2001. 
To run Carla-server, if you use built/compiled version(offered by carla.org) the following command in terminal should be executed to open CarlaUE4.sh

./CarlaUE4.sh Town02 -carla-server -windowed -X=50px -Y=40p -benchmark -fps=20


And when Carla is made from source code manually(not a compiled version), it would not contain CarlaUE4.sh file and UnrealEditor should be opened as server. And as it open the editor, it would be slower than the compiled version. To open UE4Editor as carla-server, following command should be executed. CarlaUE4.uproject is the project made by Carla and this command run this project on UE4Editor.

~/UnrealEngine_4.21/Engine/Binaries/Linux/UE4Editor ~/carla/Unreal/CarlaUE4/CarlaUE4.uproject Town02 -benchmark -fps 20
 1877 



After running Carla-Server in another terminal read the main python script. auto-parking.py or if you just want to see the maneuver, parkingManeuver.py

#please consider that all scripts are based on python2.7... so the carla connection may not work with python3

