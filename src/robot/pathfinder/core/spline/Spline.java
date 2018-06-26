package robot.pathfinder.core.spline;

import robot.pathfinder.math.Vec2D;

public interface Spline {
	public Vec2D at(double t);
	public Vec2D derivAt(double t);
	public Vec2D secondDerivAt(double t);
}
