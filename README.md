# RobotPathfinder
[![Build Status](https://travis-ci.com/Arctos6135/RobotPathfinder.svg?branch=dev%2Fmisc%2Ftravis)](https://travis-ci.com/Arctos6135/RobotPathfinder)

Robot motion profiler/path planner for tank drive robots. Used and developed by <a href="https://github.com/Arctos6135">FRC Team 6135 (Arctos)</a>!<br>
Inspired by and partially based on <a href="https://github.com/JacisNonsense/Pathfinder">Pathfinder by Jaci Brunning</a>. <em>This project is in no way intended to be a copy or replacement for Pathfinder! Though a lot of it is based on Pathfinder, the two generators have their own pros and cons.</em><br>
Thank you to FRC Team 254 (The Cheesy Poofs) for <a href="https://youtu.be/8319J1BEHwM">their amazing video on motion control</a>!<br><br>

Some of the highlights include:
* Smooth path generation with 3 different fit types
* *Respects maximum velocity constraints for both wheels, even when turning*
* *Allows wheels to turn backwards if turns are too tight*
* Handy methods that allows the mirroring and retracing of trajectories
* Follower class that can be set to follow position, velocity, acceleration and direction
* And much more...!

***All version 1 releases are deprecated; they contain serious design oversights that make them very inaccurate. Please use a version 2 release instead.***

## Purpose
Given an array of waypoints, each with coordinates and a heading, RobotPathfinder generates a smooth path that follows all the waypoints. Then, using that path, it generates left and right wheel trajectories for a robot to follow in order to achieve the desired path.

Example path:
![Path graph](http://tylertian123.github.io/images/RobotPathfinder/path1.png)<br>
Trajectory for the example path:
![Trajectory](http://tylertian123.github.io/images/RobotPathfinder/traj1.png)<br>
These graphs are generated using <b>JMathPlot</b>. You can find its repository <a href="https://github.com/yannrichet/jmathplot">here</a>.<br>
The program that generated this trajectory can be found under the `examples` directory as `TrajectoryGenerationDemo.java`.

## Usage
In every release, there will be 4 binaries:
* `RobotPathfinder-VERSION.jar` - The basic jar that contains the library, but without some dependencies. This is the jar recommended for use on a robot. It does not contain the dependencies required for graphing.
* `RobotPathfinder-VERSION-all.jar` - The fat jar that contains the library as well as all its dependencies. 
* `Trajectory-Visualizer-VERSION.jar` - The executable jar that contains the GUI Trajectory Visualization Tool (and all its dependencies).
* `RobotPathfinder-Doc-VERSION.zip` - The zip that contains the JavaDocs for all classes and methods.

Alternatively, you can build the binaries yourself by navigating to the root directory, and running `./gradlew allArchives`. (*If you're on a Windows machine, make sure you're using PowerShell not cmd!*) The jars and JavaDoc zip can then be found under the `output` directory.

**If you're using it in the First Robotics Competition:**\
In addition to adding the library jar to your build path, you must also put a copy/simlink of the jar in the WPILib Java libraries directory, or else your robot program will not compile! (This is `C:\Users\USERNAME\wpilib\user\java\lib` on Windows and `/home/USERNAME/wpilib/user/java/lib` on UNIX-based systems.)

Setup instructions for Eclipse:
* Put the jar in a folder somewhere in the project, e.g. `lib`
* Expand the project in Eclipse, right-click Referenced Libraries, Build Path -> Configure Build Path
* In the dialog that pops up, click Add Jars, navigate to and select the library jar, and confirm

Setup instructions for GradleRIO:
* Put the jar in a folder somewhere in the project, e.g. `lib`
* In `build.gradle`, under `dependencies`, add this line: `compile files('path/to/jar')`
* Your new `dependencies` should look something like this:
```groovy
dependencies {
    compile files('lib/RobotPathfinder-2.1.0.jar')
    compile wpilib()
    compile ctre()
    compile navx()
    compile openrio.powerup.matchData()
}
```

## Documentation
All classes and methods are documented with JavaDocs, in `RobotPathfinder-Doc-VERSION.zip`.\
Examples can be found under the `examples` directory.<br><br>

## GUI Trajectory Visualization Tool
A GUI Trajectory Visualization Tool in the form of an executable jar is included with every release.
![Trajectory Visualization Tool](http://tylertian123.github.io/images/RobotPathfinder/tvtool1.png)<br>

This tool generates path and trajectory graphs from waypoints and robot specification parameters; it can be used to preview paths and trajectories and check if a path is possible without having to write code for it. 

Features include:
* Creating path and trajectory graphs
* Saving waypoint data and robot specification parameters (CSV)
* Code generation

## Building
This project uses Gradle as the build system. From the project root directory, running `./gradlew allArchives` will build the project and generate the jars and a zipped JavaDoc under the `/output/` directory.
Alternatively, here are a list of tasks:
* `./gradlew build` builds the project and generates the library jar under `/build/libs/`
* `./gradlew visualizerJar` generates the Trajectory Visualization Tool jar under `/build/libs/`
* `./gradlew copyJars` copies the generated jars to `/output/`
* `./gradlew javadoc` generates JavaDocs under `/build/docs/`
* `./gradlew zipDoc` zips the generated docs into `/output/`
