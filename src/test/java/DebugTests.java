
import java.io.IOException;

import com.arctos6135.robotpathfinder.tests.core.trajectory.TankDriveTrajectoryTest;
import com.arctos6135.robotpathfinder.tests.motionprofile.MotionProfileTest;

import org.junit.runner.JUnitCore;
import org.junit.runner.Result;

public class DebugTests {

    public static void prompt() throws IOException {
        System.out.println("Press enter to continue execution");
        System.out.println("PID: " + ProcessHandle.current().pid());
        System.out.println("To Debug:\ngdb -p " + ProcessHandle.current().pid());
        System.in.read();
    }

    public static void main(String[] args) throws Exception {
        prompt();
        JUnitCore core = new JUnitCore();
        @SuppressWarnings("unused")
        Result result = core.run(MotionProfileTest.class,
                TankDriveTrajectoryTest.class);
    }
}
