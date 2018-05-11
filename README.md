# RobotPathfinder
Experimental robot motion profiler/path planner, using B&#xe9;ziers for path generation. Used on the robot of <a href="https://github.com/Arctos6135">FRC Team 6135 (Arctos)</a><br>
Inspired and partially <a href="https://github.com/JacisNonsense/Pathfinder">Pathfinder by Jaci Brunning</a>.<br>
This is just a passion project I created because I want to investigate how Pathfinder works.<br>
<br>
Thank you to FRC Team 254 (The Cheesy Poofs) for <a href="https://youtu.be/8319J1BEHwM">their amazing video on motion control</a>!

## Usage
All classes and methods are (or are soon to be) documented with JavaDocs.<br><br>
The main goal of this project is to be able to generate a smooth and fast trajectory for a robot (a tank/skid-steer drive train in particular), given a number of waypoints and the desired direction at each point.<br>
The class that does this is `robot.pathfinder.TankDriveTrajectory`:
```java
public TankDriveTrajectory(Waypoint[] waypoints, double maxVelocity, double maxAcceleration, double baseWidth, double alpha, int segmentCount)
```
This class takes in an array of waypoints and other constraints and parameters, and generates a path and trajectory. For more information on what the arguments represent, refer to the JavaDoc.<br>
After the generation is complete, the desired state of the wheels at each moment in time is obtainable through the methods `getLeft(t)` and `getRight(t)` (or `getLeftSmooth(t)` and `getRightSmooth(t)`). The trajectory at the specified time is returned as a `Moment` object, which represent a moment in time, and has methods for getting the desired distance, velocity and acceleration.<br><br>
If you then graph the path and trajectory, you'll have something that looks like this:<br>
For the path:
![Path graph](http://tylertian123.github.io/images/RobotPathfinder/generatedpath2.png)<br>
And for the trajectory:
![Trajectory](http://tylertian123.github.io/images/RobotPathfinder/generatedtrajectory2.png)<br>
Note that the distance between two wheels seem to be nonconstant in the first graph, but that caused by the scale on the X and Y axes being different.<br>
The program that generated this trajectory can be found at `robot/pathfinder/examples/Example1.java`.

## GUI Trajectory Visualization Tool
A GUI trajectory visualization tool is also included:
![Trajectory Visualization Tool](http://tylertian123.github.io/images/RobotPathfinder/trajectoryvisualizationtool.png)<br>
An executable jar for it can be found under `jars/Trajectory_Visualizer.jar`.<br><br>
The GUI tool can save and load configurations as `.csv` files, and generate graphs for the path and trajectory:
![Path graph](http://tylertian123.github.io/images/RobotPathfinder/generatedpath1.png)<br>
![Trajectory](http://tylertian123.github.io/images/RobotPathfinder/generatedtrajectory1.png)<br><br>
These graphs are generated using <b>JMathPlot</b>. You can find its repository <a href="https://github.com/yannrichet/jmathplot">here</a>.<br>
