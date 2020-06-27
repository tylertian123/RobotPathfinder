# RobotPathfinder
[![master Build Status](https://img.shields.io/travis/com/tylertian123/RobotPathfinder/master.svg?label=build%20%28master%29&logo=travis)](https://travis-ci.com/tylertian123/RobotPathfinder/branches)&nbsp;
[![Latest Build Status](https://img.shields.io/travis/com/tylertian123/RobotPathfinder.svg?label=build%20%28latest%29&logo=travis)](https://travis-ci.com/tylertian123/RobotPathfinder)&nbsp;
[![MIT License](https://img.shields.io/github/license/tylertian123/RobotPathfinder.svg?logo=data:image/svg%2bxml;base64,PD94bWwgdmVyc2lvbj0iMS4wIiBzdGFuZGFsb25lPSJubyI/Pgo8IURPQ1RZUEUgc3ZnIFBVQkxJQyAiLS8vVzNDLy9EVEQgU1ZHIDIwMDEwOTA0Ly9FTiIKICJodHRwOi8vd3d3LnczLm9yZy9UUi8yMDAxL1JFQy1TVkctMjAwMTA5MDQvRFREL3N2ZzEwLmR0ZCI+CjxzdmcgdmVyc2lvbj0iMS4wIiB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciCiB3aWR0aD0iMzIuMDAwMDAwcHQiIGhlaWdodD0iMzIuMDAwMDAwcHQiIHZpZXdCb3g9IjggOCAxNi4wMDAwMDAgMTYuMDAwMDAwIgogcHJlc2VydmVBc3BlY3RSYXRpbz0ieE1pZFlNaWQgbWVldCI+Cgo8ZyB0cmFuc2Zvcm09InRyYW5zbGF0ZSgwLjAwMDAwMCwzMi4wMDAwMDApIHNjYWxlKDAuMTAwMDAwLC0wLjEwMDAwMCkiCmZpbGw9IiNGRkZGRkYiIHN0cm9rZT0ibm9uZSI+CjxwYXRoIGQ9Ik0xMzkgMjIzIGMtMSAtNCAtMSAtMTEgMCAtMTUgMCAtNCAtMTAgLTEwIC0yMiAtMTMgLTEzIC00IC0yNCAtMTIKLTI1IC0xOCAwIC03IC00IC0yMiAtNyAtMzQgLTYgLTIwIC00IC0yMyAxOSAtMjMgMjggMCAzMiAxMCAxNiA0MCAtNyAxMyAtNgoxOSA1IDI0IDIzIDggMjEgLTcwIC0yIC04MyAtMTQgLTggLTggLTEwIDI3IC0xMCAzNSAwIDQxIDIgMjggMTAgLTI0IDEzIC0yNgo5MSAtMyA4MyAxMSAtNSAxMiAtMTEgNSAtMjQgLTE2IC0zMCAtMTIgLTQwIDE2IC00MCAyMyAwIDI1IDMgMTkgMjIgLTMgMTMgLTcKMjggLTcgMzUgLTEgNiAtMTIgMTQgLTI1IDE4IC0xMiAzIC0yMiAxMCAtMjAgMTYgMSA3IC00IDE0IC0xMCAxNiAtNyAzIC0xNCAxCi0xNCAtNHogbS0yNCAtNzMgYzMgLTUgLTEgLTEwIC0xMCAtMTAgLTkgMCAtMTMgNSAtMTAgMTAgMyA2IDggMTAgMTAgMTAgMiAwCjcgLTQgMTAgLTEweiBtOTAgMCBjMyAtNSAtMSAtMTAgLTEwIC0xMCAtOSAwIC0xMyA1IC0xMCAxMCAzIDYgOCAxMCAxMCAxMCAyCjAgNyAtNCAxMCAtMTB6Ii8+CjwvZz4KPC9zdmc+Cg==)](https://github.com/tylertian123/RobotPathfinder/blob/master/LICENSE)&nbsp;
[![GitHub Downloads (All Releases)](https://img.shields.io/github/downloads/tylertian123/RobotPathfinder/total.svg?logo=data:image/svg%2bxml;base64,PD94bWwgdmVyc2lvbj0iMS4wIiBlbmNvZGluZz0iaXNvLTg4NTktMSI/Pg0KPCEtLSBHZW5lcmF0b3I6IEFkb2JlIElsbHVzdHJhdG9yIDE2LjAuMCwgU1ZHIEV4cG9ydCBQbHVnLUluIC4gU1ZHIFZlcnNpb246IDYuMDAgQnVpbGQgMCkgIC0tPg0KPCFET0NUWVBFIHN2ZyBQVUJMSUMgIi0vL1czQy8vRFREIFNWRyAxLjEvL0VOIiAiaHR0cDovL3d3dy53My5vcmcvR3JhcGhpY3MvU1ZHLzEuMS9EVEQvc3ZnMTEuZHRkIj4NCjxzdmcgdmVyc2lvbj0iMS4xIiBpZD0iQ2FwYV8xIiB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHhtbG5zOnhsaW5rPSJodHRwOi8vd3d3LnczLm9yZy8xOTk5L3hsaW5rIiB4PSIwcHgiIHk9IjBweCINCgkgd2lkdGg9IjQzMy41cHgiIGhlaWdodD0iNDMzLjVweCIgdmlld0JveD0iMCAwIDQzMy41IDQzMy41IiBzdHlsZT0iZW5hYmxlLWJhY2tncm91bmQ6bmV3IDAgMCA0MzMuNSA0MzMuNTsiIHhtbDpzcGFjZT0icHJlc2VydmUiDQoJPg0KPGc+DQoJPGcgaWQ9ImZpbGUtZG93bmxvYWQiIGZpbGw9IiNGRkZGRkYiPg0KCQk8cGF0aCBkPSJNMzk1LjI1LDE1M2gtMTAyVjBoLTE1M3YxNTNoLTEwMmwxNzguNSwxNzguNUwzOTUuMjUsMTUzeiBNMzguMjUsMzgyLjV2NTFoMzU3di01MUgzOC4yNXoiLz4NCgk8L2c+DQo8L2c+DQo8L3N2Zz4NCg==)](https://github.com/tylertian123/RobotPathfinder/releases)&nbsp;
[![Latest Stable Release](https://img.shields.io/github/tag/tylertian123/RobotPathfinder.svg?label=release%20%28stable%29&logo=data:image/svg%2bxml;base64,PD94bWwgdmVyc2lvbj0iMS4wIiBzdGFuZGFsb25lPSJubyI/Pgo8IURPQ1RZUEUgc3ZnIFBVQkxJQyAiLS8vVzNDLy9EVEQgU1ZHIDIwMDEwOTA0Ly9FTiIKICJodHRwOi8vd3d3LnczLm9yZy9UUi8yMDAxL1JFQy1TVkctMjAwMTA5MDQvRFREL3N2ZzEwLmR0ZCI+CjxzdmcgdmVyc2lvbj0iMS4wIiB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciCiB3aWR0aD0iMzIuMDAwMDAwcHQiIGhlaWdodD0iMzIuMDAwMDAwcHQiIHZpZXdCb3g9IjggOCAxNi4wMDAwMDAgMTYuMDAwMDAwIgogcHJlc2VydmVBc3BlY3RSYXRpbz0ieE1pZFlNaWQgbWVldCI+Cgo8ZyB0cmFuc2Zvcm09InRyYW5zbGF0ZSgwLjAwMDAwMCwzMi4wMDAwMDApIHNjYWxlKDAuMTAwMDAwLC0wLjEwMDAwMCkiCmZpbGw9IiNGRkZGRkYiIHN0cm9rZT0ibm9uZSI+CjxwYXRoIGQ9Ik05NyAyMjMgYy0xOCAtMTcgLTUgLTYxIDI2IC05MSAzOSAtMzggNDcgLTM5IDc1IC05IDI5IDMxIDI4IDM3IC0xMQo3NSAtMzEgMzAgLTc0IDQyIC05MCAyNXogbTgwIC0zNSBsMzMgLTMyIC0yMiAtMjMgLTIyIC0yMyAtMzMgMzIgYy0zMSAzMCAtNDEKNTcgLTI2IDcxIDE0IDE1IDM4IDYgNzAgLTI1eiIvPgo8cGF0aCBkPSJNMTEwIDIwMCBjMCAtNSA1IC0xMCAxMCAtMTAgNiAwIDEwIDUgMTAgMTAgMCA2IC00IDEwIC0xMCAxMCAtNSAwCi0xMCAtNCAtMTAgLTEweiIvPgo8L2c+Cjwvc3ZnPgo=)](https://github.com/tylertian123/RobotPathfinder/releases/latest)&nbsp;
[![Latest (pre-)Release](https://img.shields.io/github/tag-pre/tylertian123/RobotPathfinder.svg?label=release%20%28latest%29&logo=data:image/svg%2bxml;base64,PD94bWwgdmVyc2lvbj0iMS4wIiBzdGFuZGFsb25lPSJubyI/Pgo8IURPQ1RZUEUgc3ZnIFBVQkxJQyAiLS8vVzNDLy9EVEQgU1ZHIDIwMDEwOTA0Ly9FTiIKICJodHRwOi8vd3d3LnczLm9yZy9UUi8yMDAxL1JFQy1TVkctMjAwMTA5MDQvRFREL3N2ZzEwLmR0ZCI+CjxzdmcgdmVyc2lvbj0iMS4wIiB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciCiB3aWR0aD0iMzIuMDAwMDAwcHQiIGhlaWdodD0iMzIuMDAwMDAwcHQiIHZpZXdCb3g9IjggOCAxNi4wMDAwMDAgMTYuMDAwMDAwIgogcHJlc2VydmVBc3BlY3RSYXRpbz0ieE1pZFlNaWQgbWVldCI+Cgo8ZyB0cmFuc2Zvcm09InRyYW5zbGF0ZSgwLjAwMDAwMCwzMi4wMDAwMDApIHNjYWxlKDAuMTAwMDAwLC0wLjEwMDAwMCkiCmZpbGw9IiNGRkZGRkYiIHN0cm9rZT0ibm9uZSI+CjxwYXRoIGQ9Ik05NyAyMjMgYy0xOCAtMTcgLTUgLTYxIDI2IC05MSAzOSAtMzggNDcgLTM5IDc1IC05IDI5IDMxIDI4IDM3IC0xMQo3NSAtMzEgMzAgLTc0IDQyIC05MCAyNXogbTgwIC0zNSBsMzMgLTMyIC0yMiAtMjMgLTIyIC0yMyAtMzMgMzIgYy0zMSAzMCAtNDEKNTcgLTI2IDcxIDE0IDE1IDM4IDYgNzAgLTI1eiIvPgo8cGF0aCBkPSJNMTEwIDIwMCBjMCAtNSA1IC0xMCAxMCAtMTAgNiAwIDEwIDUgMTAgMTAgMCA2IC00IDEwIC0xMCAxMCAtNSAwCi0xMCAtNCAtMTAgLTEweiIvPgo8L2c+Cjwvc3ZnPgo=)](https://github.com/tylertian123/RobotPathfinder/releases)
[![GitHub Stars](https://img.shields.io/github/stars/tylertian123/RobotPathfinder.svg?style=social)](https://github.com/tylertian123/RobotPathfinder/stargazers)&nbsp;

Robot motion profiler/path planner for tank drive (aka skid-steer or differential drive) robots. 
Developed by Tyler Tian for use by [FRC Team 6135 (Arctos)](https://arctos6135.com).
Inspired by and partially based on [Pathfinder by Jaci Brunning](https://github.com/JacisNonsense/Pathfinder). 

RobotPathfinder is intended as an improvement over Pathfinder for tank drivetrains. As such, it does not have support for other drivetrains, but has these features:
* Smooth path generation with 3 different fit types
* *Respects maximum velocity constraints for both wheels, even when turning*
* *Allows wheels to turn backwards if turns are too tight*
* *Simple motion profiles that can be re-generated on-the-fly for maximum speed and accuracy*
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
* v3 is still in its alpha stage. It uses native code to speed up execution and is on average 4 times as fast as v2. The API is still undergoing rapid development, so usage at this moment is not recommended unless you know what you're doing.

## Usage
Every release contains 3 jars, 1 zip, and numerous dynamic libraries:
* `RobotPathfinder-(VERSION).jar` - The basic jar that contains the library, but without some dependencies. This is the jar recommended for use on a robot. It does not contain the dependencies required for graphing.
* `RobotPathfinder-(VERSION)-all.jar` - The fat jar that contains the library as well as all its dependencies (JMathPlot, Gson). 
* `Trajectory-Visualizer-(VERSION).jar` - The executable jar that contains the GUI Trajectory Visualization Tool (and all its dependencies).
* `RobotPathfinder-Doc-(VERSION).zip` - The zip that contains the Javadocs for all classes and methods.
* `libRobotPathfinder-(PLATFORM).so` - **(In v3 only)** The native dynamic library used by RobotPathfinder for a specific Linux arch.
* `RobotPathfinder-(PLATFORM).dll` - **(In v3 only)** The native dynamic library used by RobotPathfinder for a specific Windows arch.

Alternatively, you can build the binaries yourself by navigating to the root directory, and running `./gradlew allArchives --rerun-tasks`. (*If you're on a Windows machine, make sure you're using PowerShell not cmd!*) The archives can then be found under the `archives` directory.

**In order to use RobotPathfinder v3, the correct dynamic library for your platform must be present in either the directory java is invoked from, or the library path.**

## FRC Usage
### GradleRIO
* Put the jar and **(v3 only)** the native library for roboRIO in a folder somewhere in the project, e.g. `lib` (**if using v3, make sure to rename the dynamic library from `libRobotPathfinder-roboRIO.so` to just `libRobotPathfinder.so`!**)
* In `build.gradle`, under `dependencies`, add this line: `compile files('path/to/jar')` and **(v3 only)** `nativeLib files('path/to/native-lib')`
* Your new `dependencies` should look something like this:
```groovy
dependencies {
    compile wpi.deps.wpilib()
    compile wpi.deps.vendor.java()

    compile files('lib/RobotPathfinder-3.0.0-alpha.0.jar')
    nativeLib files('lib/libRobotPathfinder.so') // v3 only
    
    nativeZip wpi.deps.vendor.jni(wpi.platforms.roborio)
    nativeDesktopZip wpi.deps.vendor.jni(wpi.platforms.desktop)
    testCompile 'junit:junit:4.12'
}
```

### Eclipse
* Put the jar in a folder somewhere in the project, e.g. `lib`
* Expand the project in Eclipse, right-click Referenced Libraries, Build Path -> Configure Build Path
* In the dialog that pops up, click Add Jars, navigate to and select the library jar, and confirm
* *Put a copy of the jar or a simlink to it in the WPILib Java libraries directory (`C:\Users\USERNAME\wpilib\user\java\lib` on Windows and `/home/USERNAME/wpilib/user/java/lib` on UNIX-based systems).*
##### For RobotPathfinder v3 an extra step is needed to copy the dynamic library.
* In a terminal, copy over the dynamic library using `scp`:
```shell
scp /path/to/dynamic/library lvuser@10.TE.AM.1:/usr/lib/libRobotPathfinder.so
```
*where TE.AM is your team number (e.g. for team 6135 this would be 61.35). Make sure you're connected to the robot's wifi when doing this!*

## Documentation
All classes and methods are documented with Javadocs, in `RobotPathfinder-Doc-(VERSION).zip`. The Javadocs of most versions are also available online [here](https://tylertian123.github.io/RobotPathfinder/index.html).
Head over to the [wiki](https://github.com/tylertian123/RobotPathfinder/wiki) for tutorials and examples!
***Wiki pages for RobotPathfinder v3 are coming soon.***

## GUI Trajectory Visualization Tool
A GUI Trajectory Visualization Tool in the form of an executable jar is included with every release.
![Trajectory Visualization Tool](https://user-images.githubusercontent.com/32781310/51583456-a5fd0780-1e9e-11e9-833a-e62376f82ec5.png)<br>

This tool generates path and trajectory graphs from waypoints and robot specification parameters; it can be used to preview paths and trajectories and check if a path is possible without having to write code for it. 

Features include:
* Creating path and trajectory graphs
* Saving waypoint data and robot specification parameters (CSV)
* Code generation

***To run the Trajectory Visualizer V3, make sure that you have the correct dynamic library on your library path or in the directory where java is invoked!***

## Building
This project uses Gradle as the build system. From the project root directory, running `./gradlew allArchives` will build the project and generate the jars and a zipped Javadoc under the `/archives/` directory.
Alternatively, here are a list of commonly used tasks:
* `./gradlew build` builds the project and generates the library jar under `/build/libs/`
* `./gradlew test` runs all unit tests
* `./gradlew visualizerJar` generates the Trajectory Visualization Tool jar under `/build/libs/`
* `./gradlew copyJars` copies the generated jars to `/archives/`
* `./gradlew javadoc` generates Javadocs under `/build/docs/javadoc`
* **(In v3 only)** `./gradlew testJavadoc` generates Javadocs for the test classes, under `/build/docs/testJavadoc`
* **(In v3 only)** `./gradlew completeJavadoc` generates Javadocs all classes (main and test), under `/build/docs/completeJavadoc`
* `./gradlew zipDoc` zips the generated docs into `/archives/`
* `./gradlew allArchives` builds all the archives and copies it into `/archives/`
* **(In v3 only)** `./gradlew updateJNIHeaders` will re-generate the JNI headers and copy them to `/src/main/cpp/include/jni`
* **(In v3 only)** `./gradlew copyLibDebug` will copy the debug dynamic library for the current platform to the root project folder
* **(In v3 only)** `./gradlew copyLibRelease` will copy the release dynamic library for the current platform to the root project folder
* **(In v3 only)** `./gradlew jacocoTestReport` generates a code coverage report for the test task using JaCoco

For all tasks, see the archives of `./gradlew tasks`. Note that the archives directory can be changed by changing the `archiveDir` property of the project. For example, `./gradlew allArchives -ParchiveDir=myArchiveDir` will put all the archives under `myArchiveDir`.

## Note
All classes are currently under the top-level package `com.arctos6135.robotpathfinder`.
This is because when the package was decided, the library was being developed solely for use by Arctos 6135 and under the [Arctos6135](https://github.com/Arctos6135/) organization.
It was later transferred under my name ~~because nobody else on the team has actually worked on the project at all whatsoever~~, and to keep the API consistent, I decided not to change the packages.
