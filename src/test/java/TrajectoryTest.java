import static org.junit.Assert.fail;
import static org.junit.Assert.assertThat;
import static org.hamcrest.core.Is.is;

import org.junit.Test;

import robot.pathfinder.core.RobotSpecs;
import robot.pathfinder.core.TrajectoryParams;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.WaypointEx;
import robot.pathfinder.core.path.PathType;
import robot.pathfinder.core.trajectory.BasicMoment;
import robot.pathfinder.core.trajectory.BasicTrajectory;
import robot.pathfinder.core.trajectory.TankDriveMoment;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;

public class TrajectoryTest {
    
    @Test
    public void testVelocityAndAccelerationLimitBasic() {
        RobotSpecs robotSpecs = new RobotSpecs(5.0, 3.5, 2.0);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] {
            new Waypoint(0.0, 0.0, Math.PI / 2),
            new Waypoint(0.0, 100.0, Math.PI / 2),
        };
        params.alpha = 40.0;
        params.segmentCount = 1000;
        params.isTank = true;
        params.pathType = PathType.QUINTIC_HERMITE;
        BasicTrajectory trajectory = new BasicTrajectory(robotSpecs, params);

        for(BasicMoment m : trajectory.getMoments()) {
            if(Math.abs(m.getVelocity()) > 5.0) {
                fail("The BasicTrajectory exceeded the velocity limit at time " + m.getTime());
            }
            if(Math.abs(m.getAcceleration()) > 3.5) {
                fail("The BasicTrajectory exceeded the acceleration limit at time " + m.getTime());
            }
        }
    }

    @Test
    public void testVelocityLimitTank() {
        RobotSpecs robotSpecs = new RobotSpecs(5.0, 3.5, 2.0);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] {
            new Waypoint(0.0, 0.0, Math.PI / 2),
            new Waypoint(0.0, 100.0, Math.PI / 2),
        };
        params.alpha = 40.0;
        params.segmentCount = 1000;
        params.isTank = true;
        params.pathType = PathType.QUINTIC_HERMITE;
        TankDriveTrajectory trajectory = new TankDriveTrajectory(robotSpecs, params);

        for(TankDriveMoment m : trajectory.getMoments()) {
            if(Math.abs(m.getLeftVelocity()) > 5.0) {
                fail("The left wheel of the TankDriveTrajectory exceeded the velocity limit at time " + m.getTime());
            }
            if(Math.abs(m.getRightVelocity()) > 5.0) {
                fail("The right wheel of the TankDriveTrajectory exceeded the velocity limit at time " + m.getTime());
            }
        }
    }

    @Test
    public void testBeginningAndEndWaypointEx() {
        RobotSpecs robotSpecs = new RobotSpecs(5.0, 3.5, 2.0);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] {
            new WaypointEx(0.0, 0.0, Math.PI / 2, 1.23),
            new WaypointEx(0.0, 100.0, Math.PI / 2, 3.45),
        };
        params.alpha = 40.0;
        params.segmentCount = 1000;
        params.isTank = true;
        params.pathType = PathType.QUINTIC_HERMITE;
        BasicTrajectory trajectory = new BasicTrajectory(robotSpecs, params);

        assertThat(trajectory.get(0).getVelocity(), is(1.23));
        assertThat(trajectory.get(trajectory.totalTime()).getVelocity(), is(3.45));
    }

    @Test
    public void testTankDriveTrajectoryIsSmooth() {
        {
            RobotSpecs robotSpecs = new RobotSpecs(120.0, 80.0, 25.716);
            TrajectoryParams params = new TrajectoryParams();
            params.waypoints = new Waypoint[] {
                new Waypoint(0.0, 0.0, Math.PI / 2),
                new Waypoint(50.0, 50.0, Math.PI / 4),
            };
            params.alpha = 50.0;
            params.segmentCount = 500;
            params.isTank = true;
            params.pathType = PathType.QUINTIC_HERMITE;
            TankDriveTrajectory trajectory = new TankDriveTrajectory(robotSpecs, params);

            assertThat(trajectory.isSmooth(), is(true));
        }
        {
            RobotSpecs robotSpecs = new RobotSpecs(120.0, 80.0, 25.716);
            TrajectoryParams params = new TrajectoryParams();
            params.waypoints = new Waypoint[] {
                new Waypoint(0.0, 0.0, Math.PI / 2),
                new Waypoint(50.0, 50.0, 3 * Math.PI / 4),
            };
            params.alpha = 100.0;
            params.segmentCount = 500;
            params.isTank = true;
            params.pathType = PathType.QUINTIC_HERMITE;
            TankDriveTrajectory trajectory = new TankDriveTrajectory(robotSpecs, params);

            assertThat(trajectory.isSmooth(), is(false));
        }
    }

    // TODO: Add more test cases!
}
