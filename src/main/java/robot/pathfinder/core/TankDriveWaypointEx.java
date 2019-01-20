package robot.pathfinder.core;

import robot.pathfinder.math.Vec2D;

// TODO: Add docs
public class TankDriveWaypointEx extends WaypointEx {

    final double leftVelocity, rightVelocity;

    public TankDriveWaypointEx(double x, double y, double heading, double leftVelocity, double rightVelocity) {
        // The overall velocity is just the average of the left and right
        super(x, y, heading, (leftVelocity + rightVelocity) / 2);

        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
    }

    public TankDriveWaypointEx(Vec2D location, double heading, double leftVelocity, double rightVelocity) {
        super(location, heading, (leftVelocity + rightVelocity) / 2);
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
    }

    public double getLeftVelocity() {
        return leftVelocity;
    }

    public double getRightVelocity() {
        return rightVelocity;
    }
}
