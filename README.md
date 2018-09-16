# RobotPathfinder
Robot motion profiler/path planner for tank drive robots. Used on the robot of <a href="https://github.com/Arctos6135">FRC Team 6135 (Arctos)</a><br>
Inspired by and partially based on <a href="https://github.com/JacisNonsense/Pathfinder">Pathfinder by Jaci Brunning</a>. <em>This project is in no way intended to be a copy or replacement for Pathfinder! Though a lot of it is based on Pathfinder, the two generators have their own pros and cons.</em><br>
Thank you to FRC Team 254 (The Cheesy Poofs) for <a href="https://youtu.be/8319J1BEHwM">their amazing video on motion control</a>!<br><br>

Some of the highlights include:
* Smooth path generation with 3 different fit types
* *Respects maximum velocity constraints for both wheels, even when turning*
* *Allows wheels to turn backwards if turns are too tight*
* Handy methods that allows the mirroring and retracing of trajectories
* Follower class that can be set to follow position, velocity, acceleration and direction
* And much more...!

***Version 1 is deprecated; they contain serious design oversights that make them very inaccurate. Please use a Version 2 release instead.***

## Purpose
Given an array of waypoints, each with coordinates and a heading, RobotPathfinder generates a smooth path that follows all the waypoints. Then, using that path, it generates left and right wheel trajectories for a robot to follow in order to achieve the desired path.

Example path:
![Path graph](http://tylertian123.github.io/images/RobotPathfinder/path1.png)<br>
Trajectory for the example path:
![Trajectory](http://tylertian123.github.io/images/RobotPathfinder/traj1.png)<br>
These graphs are generated using <b>JMathPlot</b>. You can find its repository <a href="https://github.com/yannrichet/jmathplot">here</a>.<br>
The program that generated this trajectory can be found under the `examples` directory as `TrajectoryGenerationDemo.java`.

## Usage
Download the jar at `build/RobotPathfinder.jar` and add it to your class path to start using RobotPathfinder!\
Note that JMathPlot is required if you wish to use any graphing features; its jars can be found under the `lib` directory.\
Additionally, if you wish to use it in a WPILib Robot Java Project, in addition to adding the jar to the classpath, you must put a copy of the jar under the WPILib Java libraries directory (`C:\Users\USERNAME\wpilib\user\java\lib` on Windows and `/home/USERNAME/wpilib/user/java/lib` on UNIX).\
Using a symlink is highly recommended, because if you ever want to update the library you only have to change one jar. For Windows, this command is `mklink path\to\symlink path\to\target`, and for UNIX it is `ln -s path/to/target path/to/symlink`.

## Documentation
All classes and methods are documented with JavaDocs. You can find the docs under the `build` directory, as `doc.zip`.\
Examples can be found under the `examples` directory.<br><br>

## GUI Trajectory Visualization Tool
A GUI Trajectory Visualization Tool in the form of an executable jar can be found at `build/Trajectory_Visualizer.jar`.
![Trajectory Visualization Tool](http://tylertian123.github.io/images/RobotPathfinder/tvtool1.png)<br>

This tool generates path and trajectory graphs from waypoints and robot specification parameters; it can be used to preview paths and trajectories and check if a path is possible without having to write code for it. 

Features include:
* Creating path and trajectory graphs
* Saving waypoint data and robot specification parameters
* Code generation
