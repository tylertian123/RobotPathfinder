import static org.hamcrest.core.Is.is;
import static org.junit.Assert.assertThat;
import static org.junit.Assert.fail;

import com.arctos6135.robotpathfinder.core.JNITrajectoryParams;
import com.arctos6135.robotpathfinder.core.JNIWaypoint;
import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.path.PathType;
import com.arctos6135.robotpathfinder.core.trajectory.BasicMoment;
import com.arctos6135.robotpathfinder.core.trajectory.JNIBasicTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.JNITankDriveTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;

import org.junit.Test;

public class TrajectoryTest {
    
    @Test
    public void testVelocityAndAccelerationLimitBasic() {
        RobotSpecs robotSpecs = new RobotSpecs(5.0, 3.5, 2.0);
        JNITrajectoryParams params = new JNITrajectoryParams();
        params.waypoints = new JNIWaypoint[] {
            new JNIWaypoint(0.0, 0.0, Math.PI / 2),
            new JNIWaypoint(0.0, 100.0, Math.PI / 2),
        };
        params.alpha = 40.0;
        params.sampleCount = 1000;
        params.isTank = true;
        params.pathType = PathType.QUINTIC_HERMITE;
        JNIBasicTrajectory trajectory = new JNIBasicTrajectory(robotSpecs, params);

        for(BasicMoment m : trajectory.getMoments()) {
            if(Math.abs(m.getVelocity()) > 5.0) {
                fail("The JNIBasicTrajectory exceeded the velocity limit at time " + m.getTime());
            }
            if(Math.abs(m.getAcceleration()) > 3.5) {
                fail("The JNIBasicTrajectory exceeded the acceleration limit at time " + m.getTime());
            }
        }
        trajectory.close();
    }

    @Test
    public void testVelocityLimitTank() {
        RobotSpecs robotSpecs = new RobotSpecs(5.0, 3.5, 2.0);
        JNITrajectoryParams params = new JNITrajectoryParams();
        params.waypoints = new JNIWaypoint[] {
            new JNIWaypoint(0.0, 0.0, Math.PI / 2),
            new JNIWaypoint(0.0, 100.0, Math.PI / 2),
        };
        params.alpha = 40.0;
        params.sampleCount = 1000;
        params.isTank = true;
        params.pathType = PathType.QUINTIC_HERMITE;
        JNITankDriveTrajectory trajectory = new JNITankDriveTrajectory(robotSpecs, params);

        for(TankDriveMoment m : trajectory.getMoments()) {
            if(Math.abs(m.getLeftVelocity()) > 5.0) {
                fail("The left wheel of the TankDriveTrajectory exceeded the velocity limit at time " + m.getTime());
            }
            if(Math.abs(m.getRightVelocity()) > 5.0) {
                fail("The right wheel of the TankDriveTrajectory exceeded the velocity limit at time " + m.getTime());
            }
        }
        trajectory.close();
    }

    @Test
    public void testBeginningAndEndWaypointEx() {
        RobotSpecs robotSpecs = new RobotSpecs(5.0, 3.5, 2.0);
        JNITrajectoryParams params = new JNITrajectoryParams();
        params.waypoints = new JNIWaypoint[] {
            new JNIWaypoint(0.0, 0.0, Math.PI / 2, 1.23),
            new JNIWaypoint(0.0, 100.0, Math.PI / 2, 3.45),
        };
        params.alpha = 40.0;
        params.sampleCount = 1000;
        params.isTank = true;
        params.pathType = PathType.QUINTIC_HERMITE;
        JNIBasicTrajectory trajectory = new JNIBasicTrajectory(robotSpecs, params);

        assertThat(trajectory.get(0).getVelocity(), is(1.23));
        assertThat(trajectory.get(trajectory.totalTime()).getVelocity(), is(3.45));
        trajectory.close();
    }
}
