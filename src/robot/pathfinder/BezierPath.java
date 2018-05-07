package robot.pathfinder;

import robot.pathfinder.math.Bezier;
import robot.pathfinder.math.Vec2D;

/**
 * A robot path composed of multiple Beziers.
 * @author Tyler
 *
 */
public class BezierPath {
	
	Bezier[] beziers;
	
	Vec2D lastPoint;
	double currentDist = 0;
	double currentT = 0;
	Vec2D[] lastWheelPositions;
	double currentWheelDists[] = new double[2];
	double currentTWheels = 0;
	
	double baseRadius;
	
	/**
	 * Constructs a Bezier path using the specified waypoints.
	 * @param waypoints - The waypoints to pass through
	 * @param alpha - The smoothness of the turns; a greater value will result in smoother turns and a small value will result in tight turns.
	 */
	public BezierPath(Waypoint[] waypoints, double alpha) {
		if(waypoints.length < 2) {
			throw new IllegalArgumentException("Not enough waypoints");
		}
		beziers = new Bezier[waypoints.length - 1];
		for(int i = 0; i < beziers.length; i ++) {
			beziers[i] = Bezier.getFromHermite(waypoints[i].getPosVec(), waypoints[i + 1].getPosVec(),
					new Vec2D(Math.cos(waypoints[i].getHeading()) * alpha, Math.sin(waypoints[i].getHeading()) * alpha),
					new Vec2D(Math.cos(waypoints[i + 1].getHeading()) * alpha, Math.sin(waypoints[i + 1].getHeading()) * alpha));
		}
		
		resetIntegration();
		resetWheelIntegration();
	}
	
	/**
	 * Sets the base plate radius (distance from the center of the robot to the wheels) of the robot following
	 * this path. This value is used to compute the result from {@link BezierPath#wheelsAt(double) wheelsAt()}.
	 * @param b - The new base radius
	 */
	public void setBaseRadius(double b) {
		baseRadius = b;
	}
	/**
	 * Retrieves the base radius (distance from the center of the robot to the wheels) of the robot following
	 * this path. This value is used to compute the result from {@link BezierPath#wheelsAt(double) wheelsAt()}.
	 * @return The base radius
	 */
	public double getBaseRaidus() {
		return baseRadius;
	}
	
	/**
	 * Returns the position at a specified time in the path.
	 * @param t - A positive real number in the range 0 to 1
	 * @return The position of the path at the specified time
	 */
	public Vec2D at(double t) {
		t *= beziers.length;
		try {
			return beziers[(int) Math.floor(t)].at(t % 1.0);
		}
		catch(ArrayIndexOutOfBoundsException e) {
			return beziers[beziers.length - 1].at(t % 1.0);
		}
	}
	/**
	 * Returns the position of the wheels at a specified time in the path.
	 * @param t - A positive real number in the range 0 to 1
	 * @return The position of the wheels at the specified time; element 0 is the left wheel, element 1 is the right wheel.
	 */
	public Vec2D[] wheelsAt(double t) {
		Vec2D position = at(t);
		Vec2D derivative = derivAt(t);
		
		double heading = Math.atan2(derivative.getY(), derivative.getX());
		double leftHeading = heading + Math.PI / 2;
		double rightHeading = heading - Math.PI / 2;
		
		return new Vec2D[] {
				new Vec2D(position.getX() + Math.cos(leftHeading) * baseRadius, position.getY() + Math.sin(leftHeading) * baseRadius),
				new Vec2D(position.getX() + Math.cos(rightHeading) * baseRadius, position.getY() + Math.sin(rightHeading) * baseRadius)
		};
	}
	/**
	 * Returns the derivative at a specified time in the path.
	 * @param t - A positive real number in the range 0 to 1
	 * @return The derivative of the path at the specified time
	 */
	public Vec2D derivAt(double t) {
		t *= beziers.length;
		return beziers[(int) Math.floor(t)].derivAt(t % 1.0);
	}
	/**
	 * Returns the second derivative at a specified time in the path.
	 * @param t - A positive real number in the range 0 to 1
	 * @return The second derivative of the path at the specified time
	 */
	public Vec2D secondDerivAt(double t) {
		t *= beziers.length;
		return beziers[(int) Math.floor(t)].secondDerivAt(t % 1.0);
	}
	
	/**
	 * Integrates length of the path.
	 * @param dt - dt
	 * @return The length of the path after a time increment of dt
	 */
	public double integrateLen(double dt) {
		currentT += dt;
		if(currentT > 1) {
			return currentDist;
		}
		Vec2D currentPoint = at(currentT);
		currentDist += lastPoint.distTo(currentPoint);
		lastPoint = currentPoint;
		return currentDist;
	}
	/**
	 * Resets the integration of the length of the path.
	 */
	public void resetIntegration() {
		currentT = 0;
		lastPoint = at(0);
		currentDist = 0;
	}
	/**
	 * Retrieves the length of the path integrated by {@link BezierPath#integrateLen(double) integrateLen()}.
	 * @return The length of the path integrated by {@link BezierPath#integrateLen(double) integrateLen()}
	 */
	public double getIntegratedLen() {
		return currentDist;
	}
	
	/**
	 * Integrates length of the path of the wheels.
	 * @param dt - dt
	 * @return The length of the path after a time increment of dt
	 */
	public double[] integrateWheelLens(double dt) {
		//Mystery bug
		if(currentTWheels == 0)
			resetWheelIntegration();
		currentTWheels += dt;
		if(currentTWheels > 1)
			return currentWheelDists;
		Vec2D[] currentWheels = wheelsAt(currentTWheels);
		currentWheelDists[0] += lastWheelPositions[0].distTo(currentWheels[0]);
		currentWheelDists[1] += lastWheelPositions[1].distTo(currentWheels[1]);
		lastWheelPositions = currentWheels;
		return currentWheelDists;
	}
	/**
	 * Resets the integration of the length of the path of the wheels.
	 */
	public void resetWheelIntegration() {
		currentTWheels = 0;
		lastWheelPositions = wheelsAt(0);
		currentWheelDists = new double[2];
	}
	/**
	 * Retrieves the length of the path of the wheels integrated by {@link BezierPath#integrateWheelLens(double) integrateWheelLens()}.
	 * @return The length of the path of the wheels integrated by {@link BezierPath#integrateWheelLens(double) integrateWheelLens()}
	 */
	public double[] getIntegratedWheelLens() {
		return currentWheelDists.clone();
	}
}
