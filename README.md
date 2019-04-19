# RobotPathfinder
[![Build Status](https://travis-ci.com/Arctos6135/RobotPathfinder.svg?branch=dev%2Fjni)](https://travis-ci.com/Arctos6135/RobotPathfinder)

Robot motion profiler/path planner for tank drive (aka skid-steer or differential drive) robots. Used and developed by [FRC Team 6135 (Arctos)](https://github.com/Arctos6135).
Inspired by and partially based on [Pathfinder by Jaci Brunning](https://github.com/JacisNonsense/Pathfinder). 

RobotPathfinder is intended as an improvement over Pathfinder for tank drivetrains. As such, it does not have support for other drivetrains, but features these improvements:
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

## A Note on Versions
There are currently 3 major versions of RobotPathfinder (v1, v2 and v3):
* v1 is outdated and inaccurate. You should never use it.
* v2 has been extensively used and tested and is stable.
* v3 is still in its alpha stage. It uses native code to speed up execution and is on average 4 times as fast as v2.

## Usage
Every release contains 3 jars, 1 zip, and numerous dynamic libraries:
* `RobotPathfinder-(VERSION).jar` - The basic jar that contains the library, but without some dependencies. This is the jar recommended for use on a robot. It does not contain the dependencies required for graphing.
* `RobotPathfinder-(VERSION)-all.jar` - The fat jar that contains the library as well as all its dependencies. 
* `Trajectory-Visualizer-(VERSION).jar` - The executable jar that contains the GUI Trajectory Visualization Tool (and all its dependencies).
* `RobotPathfinder-Doc-(VERSION).zip` - The zip that contains the JavaDocs for all classes and methods.
* `libRobotPathfinder-(PLATFORM).so` - The native dynamic library used by RobotPathfinder for a specific Linux arch.
* `RobotPathfinder-(PLATFORM).dll` - The native dynamic library used by RobotPathfinder for a specific Windows arch.

Alternatively, you can build the binaries yourself by navigating to the root directory, and running `./gradlew allArchives --rerun-tasks`. (*If you're on a Windows machine, make sure you're using PowerShell not cmd!*) The archives can then be found under the `output` directory.

**In order to use RobotPathfinder, the correct dynamic library for your platform must be present in either the directory java is invoked from, or the library path.**

## FRC Usage
### GradleRIO
* Put the jar **and the native library for roboRIO** in a folder somewhere in the project, e.g. `lib` (make sure to rename the dynamic library from `libRobotPathfinder-roboRIO.so` to just `libRobotPathfinder.so`!)
* In `build.gradle`, under `dependencies`, add this line: `compile files('path/to/jar')` and `nativeLib files('path/to/native-lib')`
* Your new `dependencies` should look something like this:
```groovy
dependencies {
    compile wpi.deps.wpilib()
    compile wpi.deps.vendor.java()

    compile files('lib/RobotPathfinder-3.0.0-alpha.0.jar')
    nativeLib files('lib/libRobotPathfinder.so')
    
    nativeZip wpi.deps.vendor.jni(wpi.platforms.roborio)
    nativeDesktopZip wpi.deps.vendor.jni(wpi.platforms.desktop)
    testCompile 'junit:junit:4.12'
}
```

### Eclipse
*** This section is outdated and does not contain instructions for installing the native shared library.***
* Put the jar in a folder somewhere in the project, e.g. `lib`
* Expand the project in Eclipse, right-click Referenced Libraries, Build Path -> Configure Build Path
* In the dialog that pops up, click Add Jars, navigate to and select the library jar, and confirm
* *Put a copy of the jar or a simlink to it in the WPILib Java libraries directory (`C:\Users\USERNAME\wpilib\user\java\lib` on Windows and `/home/USERNAME/wpilib/user/java/lib` on UNIX-based systems).*

## Documentation
All classes and methods are documented with JavaDocs, in `RobotPathfinder-Doc-(VERSION).zip`.\
Head over to the [wiki](https://github.com/Arctos6135/RobotPathfinder/wiki) for tutorials and examples!
***Currently the wiki and examples are outdated. They will be updated for RobotPathfinder v3 soon.***

## GUI Trajectory Visualization Tool
A GUI Trajectory Visualization Tool in the form of an executable jar is included with every release.
![Trajectory Visualization Tool](https://user-images.githubusercontent.com/32781310/51583456-a5fd0780-1e9e-11e9-833a-e62376f82ec5.png)<br>

This tool generates path and trajectory graphs from waypoints and robot specification parameters; it can be used to preview paths and trajectories and check if a path is possible without having to write code for it. 

Features include:
* Creating path and trajectory graphs
* Saving waypoint data and robot specification parameters (CSV)
* Code generation

***To run the Trajectory Visualizer, make sure that you have the correct dynamic library on your library path or in the directory where java is invoked!***

## Building
This project uses Gradle as the build system. From the project root directory, running `./gradlew allArchives` will build the project and generate the jars and a zipped JavaDoc under the `/output/` directory.
Alternatively, here are a list of tasks:
* `./gradlew build` builds the project and generates the library jar under `/build/libs/`
* `./gradlew visualizerJar` generates the Trajectory Visualization Tool jar under `/build/libs/`
* `./gradlew copyJars` copies the generated jars to `/output/`
* `./gradlew javadoc` generates JavaDocs under `/build/docs/`
* `./gradlew zipDoc` zips the generated docs into `/output/`
* `./gradlew allArchives` builds all the archives and copies it into `/output/`
* `./gradlew updateJNIHeaders` will re-generate the JNI headers and copy them to `/src/main/cpp/include/jni`
* `./gradlew copyLibDebug` will copy the debug dynamic library for the current platform to the root project folder
* `./gradlew copyLibRelease` will copy the release dynamic library for the current platform to the root project folder

Thank you to our generous sponsors:<br/>
<img src="https://dynamicmedia.zuza.com/zz/m/original_/3/a/3aae60b3-ff18-4be5-b2b1-e244943a85fb/TDSB_Gallery.png" alt="Toronto District School Board" height="200px"/>
<img src="https://developer.nordicsemi.com/.webresources/NordicS.jpg" alt="Nordic Semiconductors" height="170px"/>
<img src="https://upload.wikimedia.org/wikipedia/en/thumb/5/50/SNC-Lavalin_logo.svg/1280px-SNC-Lavalin_logo.svg.png" alt="SNC-Lavalin" height="170px"/>
<img src="https://user-images.githubusercontent.com/32781310/52970668-acd64780-3382-11e9-857f-85b829690e0c.png" alt="Scotia McLeod" height="200px"/>
<img src="https://kissmybutton.gr/wp-content/uploads/2017/09/ryver.png" alt="Ryver Inc." height="200px"/>
<img src="https://user-images.githubusercontent.com/32781310/52224389-eaf94480-2875-11e9-82ba-78ec58cd20cd.png" alt="The Maker Bean Cafe" height="200px"/>
<img src="http://connecttech.com/logo.jpg" alt="Connect Tech Inc." height="200px"/>
<img src="https://brafasco.com/media/wysiwyg/HDS_construction_industrial_BF_4C_pos.png" alt="HD Supply Brafasco" height="200px"/>
<img src="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcRqnEGnLesUirrtMQfhxLGUTZn2xkVWpbROlvmABI2Nk6HzhD1w" alt="Arbour Memorial" height="200px"/>
