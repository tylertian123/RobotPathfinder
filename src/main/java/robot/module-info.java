module robot.pathfinder {
    requires java.base;
    requires java.desktop;
    requires jmathplot;

    exports robot.pathfinder.core;
    exports robot.pathfinder.core.path;
    exports robot.pathfinder.core.trajectory;
    exports robot.pathfinder.math;
}
