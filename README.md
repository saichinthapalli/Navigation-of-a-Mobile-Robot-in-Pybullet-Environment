# Navigation-of-a-Mobile-Robot-in-Pybullet-Environment
The mobile robot is presented in a 3D space with the 6 cubes as the obstacles. When Main.py file is run, the mobile robot starts from the point and travels until a certain point to check if there is an obstacle. The presence of obstacle sets the flag value as true and vice versa if there is no obstacle. In case of obstacle, as it will create a problem for the robot, the point from where the robot started is deleted and another point is taken into consideration. The same process is repeated until it finds the right path for the mobile robot to reach the final location. Hence, once the robot travels to the destination without encountering any obstacles then the start and end point of the robot is noted down for future reptations and various cases.