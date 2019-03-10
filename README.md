# RobotPathfinder
[![Build Status](https://travis-ci.com/Arctos6135/RobotPathfinder.svg?branch=dev%2Fjni)](https://travis-ci.com/Arctos6135/RobotPathfinder)

## RobotPathfinder is currently undergoing a massive refactoring process to convert everything into JNI to improve performance. This branch is being actively worked on.

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

## Purpose
Given an array of waypoints, each with coordinates, a heading, and optionally a velocity, RobotPathfinder generates a smooth path that follows all the waypoints. Then, using that path, it generates left and right wheel trajectories for a robot to follow in order to achieve the desired path.

Example path:
![Path graph](http://tylertian123.github.io/images/RobotPathfinder/path1.png)<br>
Trajectory for the example path:
![Trajectory](http://tylertian123.github.io/images/RobotPathfinder/traj1.png)<br>
These graphs are generated using <b>JMathPlot</b>. You can find its repository <a href="https://github.com/yannrichet/jmathplot">here</a>.<br>
The program that generated this trajectory can be found under the `examples` directory as `TrajectoryGenerationDemo.java`.

## Usage
Every release contains 4 binaries:
* `RobotPathfinder-(VERSION).jar` - The basic jar that contains the library, but without some dependencies. This is the jar recommended for use on a robot. It does not contain the dependencies required for graphing.
* `RobotPathfinder-(VERSION)-all.jar` - The fat jar that contains the library as well as all its dependencies. 
* `Trajectory-Visualizer-(VERSION).jar` - The executable jar that contains the GUI Trajectory Visualization Tool (and all its dependencies).
* `RobotPathfinder-Doc-(VERSION).zip` - The zip that contains the JavaDocs for all classes and methods.

Alternatively, you can build the binaries yourself by navigating to the root directory, and running `./gradlew allArchives --rerun-tasks`. (*If you're on a Windows machine, make sure you're using PowerShell not cmd!*) The jars and JavaDoc zip can then be found under the `output` directory.

**FRC Usage:**\
In addition to adding the library jar to your build path, you must also put a copy/simlink of the jar in the WPILib Java libraries directory, or else your robot program will not compile! (This is `C:\Users\USERNAME\wpilib\user\java\lib` on Windows and `/home/USERNAME/wpilib/user/java/lib` on UNIX-based systems.)

Setup instructions for GradleRIO:
* Put the jar in a folder somewhere in the project, e.g. `lib`
* In `build.gradle`, under `dependencies`, add this line: `compile files('path/to/jar')`
* Your new `dependencies` should look something like this:
```groovy
dependencies {
    compile wpi.deps.wpilib()
    compile wpi.deps.vendor.java()

    compile files('lib/RobotPathfinder-2.1.1.jar')
    
    nativeZip wpi.deps.vendor.jni(wpi.platforms.roborio)
    nativeDesktopZip wpi.deps.vendor.jni(wpi.platforms.desktop)
    testCompile 'junit:junit:4.12'
}
```

Setup instructions for Eclipse:
* Put the jar in a folder somewhere in the project, e.g. `lib`
* Expand the project in Eclipse, right-click Referenced Libraries, Build Path -> Configure Build Path
* In the dialog that pops up, click Add Jars, navigate to and select the library jar, and confirm
* Put a copy of the jar or a simlink to it in the WPILib Java libraries directory (`C:\Users\USERNAME\wpilib\user\java\lib` on Windows and `/home/USERNAME/wpilib/user/java/lib` on UNIX-based systems).

## Documentation
All classes and methods are documented with JavaDocs, in `RobotPathfinder-Doc-(VERSION).zip`.\
Head over to the wiki for tutorials and examples!
More examples can be found under the `examples` directory.<br><br>

## GUI Trajectory Visualization Tool
A GUI Trajectory Visualization Tool in the form of an executable jar is included with every release.
![Trajectory Visualization Tool](https://user-images.githubusercontent.com/32781310/51583456-a5fd0780-1e9e-11e9-833a-e62376f82ec5.png)<br>

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

## Thank you to our generous sponsors:
<img src="http://connecttech.com/logo.jpg" alt="Connect Tech Inc." height="200px"/><img src="https://user-images.githubusercontent.com/32781310/52970668-acd64780-3382-11e9-857f-85b829690e0c.png" alt="Scotia McLeod" height="200px"/><img src="https://kissmybutton.gr/wp-content/uploads/2017/09/ryver.png" alt="Ryver Inc." height="200px"/><img src="https://user-images.githubusercontent.com/32781310/52224389-eaf94480-2875-11e9-82ba-78ec58cd20cd.png" alt="The Maker Bean Cafe" height="200px"/><img src="https://brafasco.com/media/wysiwyg/HDS_construction_industrial_BF_4C_pos.png" alt="HD Supply Brafasco" height="200px"/><img src="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcRqnEGnLesUirrtMQfhxLGUTZn2xkVWpbROlvmABI2Nk6HzhD1w" alt="Arbour Memorial" height="200px"/><img src="https://developer.nordicsemi.com/.webresources/NordicS.jpg" alt="Nordic Semiconductors" height="170px"/><img src="https://dynamicmedia.zuza.com/zz/m/original_/3/a/3aae60b3-ff18-4be5-b2b1-e244943a85fb/TDSB_Gallery.png" alt="Toronto District School Board" height="200px"/>
