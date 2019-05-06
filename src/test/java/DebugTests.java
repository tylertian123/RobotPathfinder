
import java.io.IOException;

import javax.swing.JFrame;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.motionprofile.TrapezoidalMotionProfile;
import com.arctos6135.robotpathfinder.tools.Grapher;

public class DebugTests {

    public static void prompt() throws IOException {
        System.out.println("Press enter to continue execution");
        System.out.println("PID: " + ProcessHandle.current().pid());
        System.out.println("To Debug:\ngdb -p " + ProcessHandle.current().pid());
        System.in.read();
    }

    public static void main(String[] args) throws Exception {
        prompt();


        TrapezoidalMotionProfile p = new TrapezoidalMotionProfile(new RobotSpecs(5, 3), -3, -5);
        JFrame f = Grapher.graph(p, 0.01);
        f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        f.setVisible(true);
    }
}
