import static org.hamcrest.core.Is.is;
import static org.hamcrest.core.IsNot.not;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.TrajectoryParams;
import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.path.Path;
import com.arctos6135.robotpathfinder.core.path.PathType;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveTrajectory;

import org.junit.Test;
import static org.junit.Assert.assertThat;

public class BasicTest {
    
    @Test
    public void testPathAllMethods() {
        Waypoint[] waypoints = new Waypoint[] {
            new Waypoint(0, 0, 0),
            new Waypoint(10, 10, 0),
            new Waypoint(12.34, 5.67, Math.PI),
        };
        Path bezierPath = new Path(waypoints, 30, PathType.BEZIER);
        Path cubicPath = new Path(waypoints, 30, PathType.CUBIC_HERMITE);
        Path quinticPath = new Path(waypoints, 30, PathType.QUINTIC_HERMITE);

        assertThat(bezierPath.computeLen(100), is(not(Double.NaN)));
        assertThat(bezierPath.at(0), is(not(null)));
        assertThat(bezierPath.derivAt(0), is(not(null)));
        assertThat(bezierPath.secondDerivAt(0), is(not(null)));
        bezierPath.setBaseRadius(1);
        bezierPath.setDrivingBackwards(true);
        assertThat(bezierPath.t2S(0), is(not(Double.NaN)));
        assertThat(bezierPath.s2T(0), is(not(Double.NaN)));
        bezierPath._updateWaypoints();
        assertThat(bezierPath.mirrorLeftRight(), is(not(null)));
        assertThat(bezierPath.mirrorFrontBack(), is(not(null)));
        assertThat(bezierPath.retrace(), is(not(null)));

        bezierPath.close();
        cubicPath.close();
        quinticPath.close();
    }

    @Test
    public void testTankDriveTrajectoryAllMethods() {
        RobotSpecs specs = new RobotSpecs(5, 3, 1);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] {
            new Waypoint(0, 0, Math.PI / 2),
            new Waypoint(10, 10, Math.PI / 2),
            new Waypoint(0, 20, Math.PI),
        };
        params.alpha = 20;
        params.isTank = true;
        params.pathType = PathType.QUINTIC_HERMITE;
        params.sampleCount = 1000;

        TankDriveTrajectory traj = new TankDriveTrajectory(specs, params);
        assertThat(traj.getMoments(), is(not(null)));
        assertThat(traj.get(0), is(not(null)));
        assertThat(traj.getPath(), is(not(null)));
        assertThat(traj.mirrorFrontBack(), is(not(null)));
        assertThat(traj.mirrorLeftRight(), is(not(null)));
        assertThat(traj.retrace(), is(not(null)));
        traj.close();
    }
}
