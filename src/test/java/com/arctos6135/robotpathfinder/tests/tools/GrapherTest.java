package com.arctos6135.robotpathfinder.tests.tools;

import static org.hamcrest.Matchers.closeTo;
import static org.hamcrest.Matchers.either;
import static org.hamcrest.Matchers.instanceOf;
import static org.hamcrest.Matchers.is;
import static org.hamcrest.Matchers.lessThan;
import static org.hamcrest.Matchers.lessThanOrEqualTo;
import static org.junit.Assert.assertThat;

import java.util.function.Consumer;
import java.util.function.Function;

import javax.swing.JFrame;

import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.path.Path;
import com.arctos6135.robotpathfinder.core.path.PathType;
import com.arctos6135.robotpathfinder.core.trajectory.BasicMoment;
import com.arctos6135.robotpathfinder.core.trajectory.Moment;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.Followable;
import com.arctos6135.robotpathfinder.math.MathUtils;
import com.arctos6135.robotpathfinder.math.Vec2D;
import com.arctos6135.robotpathfinder.motionprofile.MotionProfile;
import com.arctos6135.robotpathfinder.tests.TestHelper;
import com.arctos6135.robotpathfinder.tests.core.trajectory.TrajectoryTestingUtils;
import com.arctos6135.robotpathfinder.tools.Grapher;
import com.arctos6135.robotpathfinder.util.Pair;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TestName;
import org.math.plot.Plot2DPanel;

/**
 * This class contains tests for {@link Grapher}.
 * 
 * @author Tyler Tian
 */
public class GrapherTest {

    @Rule
    public TestName testName = new TestName();

    /**
     * A fake implementation of {@link Followable} for testing.
     * 
     * @author Tyler Tian
     * @param <T> The type of moment used
     */
    public static class FakeFollowable<T extends Moment> implements Followable<T> {

        /**
         * A function that accepts time and returns a moment for that time.
         * <p>
         * Used in {@link #get()}.
         * </p>
         */
        public Function<Double, T> getCallback = null;

        /**
         * The number to return for {@link #totalTime()}.
         */
        public double fakeTotalTime = 0;

        /**
         * {@inheritDoc}
         */
        @Override
        public T get(double t) {
            return getCallback.apply(t);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public double totalTime() {
            return fakeTotalTime;
        }
    }

    /**
     * A fake implementation of {@link MotionProfile} for testing.
     * 
     * @author Tyler Tian
     */
    public static class FakeMotionProfile implements MotionProfile {

        /**
         * The number to return for {@link #totalTime()}.
         */
        public double fakeTotalTime = 0;
        /**
         * The value to return for {@link #isReversed()}.
         */
        public boolean fakeIsReversed = false;

        /**
         * A function that accepts time and returns the position for that time.
         * <p>
         * Used in {@link #position(double)}.
         * </p>
         */
        Function<Double, Double> positionCallback;
        /**
         * A function that accepts time and returns the velocity for that time.
         * <p>
         * Used in {@link #velocity(double)}.
         * </p>
         */
        Function<Double, Double> velocityCallback;
        /**
         * A function that accepts time and returns the acceleration for that time.
         * <p>
         * Used in {@link #acceleration(double)}.
         * </p>
         */
        Function<Double, Double> accelerationCallback;

        /**
         * {@inheritDoc}
         */
        @Override
        public double totalTime() {
            return fakeTotalTime;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public double position(double t) {
            return positionCallback.apply(t);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public double velocity(double t) {
            return velocityCallback.apply(t);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public double acceleration(double t) {
            return accelerationCallback.apply(t);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public boolean isReversed() {
            return fakeIsReversed;
        }

    }

    /**
     * A subclass of {@link Path} for testing.
     * <p>
     * This class inherits everything from {@link Path} as-is, except that it calls
     * a callback function when {@link Path#at(double)} or
     * {@link Path#wheelsAt(double)} is called (in addition to returning the normal
     * values).
     * </p>
     * 
     * @author Tyler Tian
     */
    private static class TestPath extends Path {

        /**
         * A callback to be called when {@link #at(double)} is called.
         * <p>
         * This should accept a single double, which represents the time.
         * </p>
         */
        Consumer<Double> atCallback;
        /**
         * A callback to be called when {@link #wheelsAt(double)} is called.
         * <p>
         * This should accept a single double, which represents the time.
         * </p>
         */
        Consumer<Double> wheelsAtCallback;

        /**
         * See docs for {@link Path#Path(Waypoint[], double, PathType)}.
         * 
         * @param waypoints See docs for {@link Path#Path(Waypoint[], double, PathType)}
         * @param alpha See docs for {@link Path#Path(Waypoint[], double, PathType)}
         * @param pathType See docs for {@link Path#Path(Waypoint[], double, PathType)}
         */
        public TestPath(Waypoint[] waypoints, double alpha, PathType pathType) {
            super(waypoints, alpha, pathType);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public Vec2D at(double time) {
            atCallback.accept(time);
            return super.at(time);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public Pair<Vec2D, Vec2D> wheelsAt(double time) {
            wheelsAtCallback.accept(time);
            return super.wheelsAt(time);
        }
    }

    /**
     * Performs basic testing on
     * {@link Grapher#graphBasicFollowable(Followable, double)}.
     * <p>
     * This test tests for whether the difference in time between two samples is
     * correct during graphing. After graphing, it tests if the returned frame has a
     * content pane that is an instance of {@link Plot2DPanel}, has the correct
     * default close operation and is not visible.
     * </p>
     */
    @Test
    public void testGrapherGraphBasicFollowable() {
        TestHelper helper = new TestHelper(getClass(), testName);

        FakeFollowable<BasicMoment> followable = new FakeFollowable<>();
        followable.fakeTotalTime = helper.getDouble("totalTime", 100000);
        var wrapper = new Object() {
            double lastTime = Double.NaN;
        };
        double dt = helper.getDouble("dt", followable.fakeTotalTime / 1000, followable.fakeTotalTime);
        followable.getCallback = (t) -> {
            assertThat(t, lessThan(followable.fakeTotalTime));
            if (!Double.isNaN(wrapper.lastTime)) {
                assertThat(t - wrapper.lastTime, closeTo(dt, MathUtils.getFloatCompareThreshold()));
            }
            wrapper.lastTime = t;

            return new BasicMoment();
        };

        JFrame frame = Grapher.graphBasicFollowable(followable, dt);

        assertThat(frame.getContentPane(), instanceOf(Plot2DPanel.class));
        assertThat(frame.getDefaultCloseOperation(), is(JFrame.DISPOSE_ON_CLOSE));
        assertThat(frame.isVisible(), is(false));
    }

    /**
     * Performs basic testing on
     * {@link Grapher#graphTankDriveFollowable(Followable, double)}.
     * <p>
     * This test tests for whether the difference in time between two samples is
     * correct during graphing. After graphing, it tests if the returned frame has a
     * content pane that is an instance of {@link Plot2DPanel}, has the correct
     * default close operation and is not visible.
     * </p>
     */
    @Test
    public void testGrapherGraphTankDriveFollowable() {
        TestHelper helper = new TestHelper(getClass(), testName);

        FakeFollowable<TankDriveMoment> followable = new FakeFollowable<>();
        followable.fakeTotalTime = helper.getDouble("totalTime", 100000);
        var wrapper = new Object() {
            double lastTime = Double.NaN;
        };
        double dt = helper.getDouble("dt", followable.fakeTotalTime / 1000, followable.fakeTotalTime);
        followable.getCallback = (t) -> {
            assertThat(t, lessThan(followable.fakeTotalTime));
            if (!Double.isNaN(wrapper.lastTime)) {
                assertThat(t - wrapper.lastTime, closeTo(dt, MathUtils.getFloatCompareThreshold()));
            }
            wrapper.lastTime = t;

            return new TankDriveMoment();
        };

        JFrame frame = Grapher.graphTankDriveFollowable(followable, dt);

        assertThat(frame.getContentPane(), instanceOf(Plot2DPanel.class));
        assertThat(frame.getDefaultCloseOperation(), is(JFrame.DISPOSE_ON_CLOSE));
        assertThat(frame.isVisible(), is(false));
    }

    /**
     * Performs basic testing on
     * {@link Grapher#graphPath(Path, double)}.
     * <p>
     * This test tests for whether the difference in time between two samples is
     * correct during graphing. After graphing, it tests if the returned frame has a
     * content pane that is an instance of {@link Plot2DPanel}, has the correct
     * default close operation and is not visible.
     * </p>
     */
    @Test
    public void testGrapherGraphPath() {
        TestHelper helper = new TestHelper(getClass(), testName);

        Waypoint[] waypoints = TrajectoryTestingUtils.getRandomWaypoints(helper);
        TestPath path = new TestPath(waypoints, helper.getDouble("alpha", 100000),
                TrajectoryTestingUtils.getRandomPathType(helper));
        var atWrapper = new Object() {
            double lastTime = Double.NaN;
        };
        var wheelsAtWrapper = new Object() {
            double lastTime = Double.NaN;
        };
        double dt = helper.getDouble("dt", 0.001, 1);
        path.atCallback = (t) -> {
            assertThat(t, lessThanOrEqualTo(1.0));
            System.out.println(t);
            if (!Double.isNaN(atWrapper.lastTime)) {
                assertThat(t,
                        either(closeTo(atWrapper.lastTime + dt, MathUtils.getFloatCompareThreshold())).or(is(1.0)));
            }
            atWrapper.lastTime = t;
        };
        path.wheelsAtCallback = (t) -> {
            assertThat(t, lessThanOrEqualTo(1.0));
            System.out.println(t);
            if (!Double.isNaN(wheelsAtWrapper.lastTime)) {
                assertThat(t, either(closeTo(wheelsAtWrapper.lastTime + dt, MathUtils.getFloatCompareThreshold()))
                        .or(is(1.0)));
            }
            wheelsAtWrapper.lastTime = t;
        };

        JFrame frame = Grapher.graphPath(path, dt);

        assertThat(frame.getContentPane(), instanceOf(Plot2DPanel.class));
        assertThat(frame.getDefaultCloseOperation(), is(JFrame.DISPOSE_ON_CLOSE));
        assertThat(frame.isVisible(), is(false));
    }

    /**
     * Performs basic testing on
     * {@link Grapher#graphMotionProfile(MotionProfile, double)}.
     * <p>
     * This test tests for whether the difference in time between two samples is
     * correct during graphing. After graphing, it tests if the returned frame has a
     * content pane that is an instance of {@link Plot2DPanel}, has the correct
     * default close operation and is not visible.
     * </p>
     */
    @Test
    public void testGrapherGraphMotionProfile() {
        TestHelper helper = new TestHelper(getClass(), testName);

        FakeMotionProfile profile = new FakeMotionProfile();
        profile.fakeTotalTime = helper.getDouble("totalTime", 100000);
        var posWrapper = new Object() {
            double lastTime = Double.NaN;
        };
        var velWrapper = new Object() {
            double lastTime = Double.NaN;
        };
        var accelWrapper = new Object() {
            double lastTime = Double.NaN;
        };
        double dt = helper.getDouble("dt", profile.fakeTotalTime / 1000, profile.fakeTotalTime);
        profile.positionCallback = (t) -> {
            assertThat(t, lessThan(profile.fakeTotalTime));
            if (!Double.isNaN(posWrapper.lastTime)) {
                assertThat(t - posWrapper.lastTime, closeTo(dt, MathUtils.getFloatCompareThreshold()));
            }
            posWrapper.lastTime = t;

            return 0.0;
        };
        profile.velocityCallback = (t) -> {
            assertThat(t, lessThan(profile.fakeTotalTime));
            if (!Double.isNaN(velWrapper.lastTime)) {
                assertThat(t - velWrapper.lastTime, closeTo(dt, MathUtils.getFloatCompareThreshold()));
            }
            velWrapper.lastTime = t;

            return 0.0;
        };
        profile.accelerationCallback = (t) -> {
            assertThat(t, lessThan(profile.fakeTotalTime));
            if (!Double.isNaN(accelWrapper.lastTime)) {
                assertThat(t - accelWrapper.lastTime, closeTo(dt, MathUtils.getFloatCompareThreshold()));
            }
            accelWrapper.lastTime = t;

            return 0.0;
        };

        JFrame frame = Grapher.graphMotionProfile(profile, dt);

        assertThat(frame.getContentPane(), instanceOf(Plot2DPanel.class));
        assertThat(frame.getDefaultCloseOperation(), is(JFrame.DISPOSE_ON_CLOSE));
        assertThat(frame.isVisible(), is(false));
    }
}
