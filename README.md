# Scullion Robot

## Abstract
  This project consists in the implementation of a robot capable of operate in a real kitchen, working alongside of cookers and providing 
them they ingredients they precise. To do so, the robot will recieve commands through a voice interface. 

  The robot has three zones:
* The ingredients zone, where all the ingredients are kept
* The dropping zone, where the robot puts the ingredients asked by the cooker
* The recovery zone, where the ingredients are put to be returned to the first zone

  In order for the robot to "see" the ingredients in the third zone there will be a camera. This element will keep track of the different ingredients
in the zone thanks to Arucos images in the top.

  The robot selected to implement this application is the Kinova Jaco, simulated in ROS and in the UA Labs.
  
  Plataforms: Linux 20.04, Noetic, (it may work in other distributions), MoveIt!

## Installation
  These instructions take into account that the person reading this README has some knowledge about how ROS works.

### Install MoveIt!
  In this project, MoveIt! is the main trajectory planner, so in order to get it, this is how it has to be installed, replacing the field $DISTRO with your
  own ROS distribution, but it may not work properly:
  
  ```
sudo apt install ros-$DISTRO-moveit
  ```


### Speech recognition
  This project uses some speech recognition features via AI. To install these libraries, follow the present instructions:
  
  ```
sudo pip3 install SpeechRecognition                  # Install Speech Recognition
sudo apt-get install python-pyaudio python3-pyaudio  # Install depedencies
sudo apt-get install libasound-dev
  ```
  
  Check the Appendix 1 for any doubts or if there are some errors related to the voice interface
  
  To get this repository and compile it in your PC:

```
mkdir catkin_ws/
cd catkin_ws
mkdir src
cd src
git clone https://github.com/DanielFrauAlfaro/Proyecto_Servicios
cd ..
catkin_make
```

  Once your have execute the prevoius commands, you will have your project compiled.
  
## Execution
  To execute this project, it is important to point out that there are different modes:
  
* Simulation: ``` roslaunch controllers scullio.launch ```
* Real: ``` roslaunch controllers scullion.launch mode:=real```
* With the tiles instead of the voice: ``` roslaunch controllers scullion.launch mode:=tiles ```

### Another ways of execution
  There are other ways of executing this project. These ones involves bash scripts and GUI.

1. Move the file `kinova.sh` to the `catkin_ws` folder
2. Allow execution permissions `sudo chmod +x kinova.sh`
3. Now it can be executed by typing `bash kinova.sh` in a Linux terminal.
   - In addition, this file can be executed by double-clicking on it. To do so, select the file and go to the three bar icon in the `Files` window. Then click in `Preferences`. After this, go to `Behavoir` and select `Run Them`


## Apendix 1
  If there are some errors with the Speech Recognition interface, follow the next steps:
  
  ```
sudo apt-get install libasound-dev
  ```
  
  Then download `portaudio` from [here](http://files.portaudio.com/download.html), and compile it with `make`.
  
  If there still are some errors, try these:
  
  ```
sudo apt-get install libasound2
sudo apt-get install libasound2:i386
  ```


## References
* [Kinova Repository](https://github.com/Kinovarobotics/kinova-ros)
* [ROS](https://www.ros.org/)
* Pynput installation: ``` pip3 install pynput ```
