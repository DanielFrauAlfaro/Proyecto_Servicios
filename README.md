# Scullion Robot

This project consists in the implementation of a robot capable of operate in a real kitchen, working alongside of cookers and providing 
them they ingredients they precise. To do so, the robot will recieve commands through a voice interface. 

The robot has three zones:
* The ingredients zone, where all the ingredients are kept
* The dropping zone, where the robot puts the ingredients asked by the cooker
* The recovery zone, where the ingredients are put to be returned to the first zone

In order for the robot to "see" the ingredients in the third zone there will be a camera. This element will keep track of the different ingredients
in the zone thanks to Arucos images in the top.
