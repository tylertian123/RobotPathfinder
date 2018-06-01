package robot.pathfinder;

import robot.pathfinder.math.Bezier;
import robot.pathfinder.math.Vec2D;

/**
 * A robot path composed of multiple Beziers.
 * @author Tyler Tian
 *
 */
public class BezierPath {
	
	Bezier[] beziers;
	
	//'i' stands for integration
	Vec2D iLastPos;
	double iCurrentDist = 0;
	double iCurrentTime = 0;
	
	//'iw' stands for integration (wheels)
	Vec2D[] iwLastPos;
	double iwCurrentDists[] = new double[2];
	double iwCurrentTime = 0;
	
	double baseRadius;
	
	Waypoint[] waypoints;
	double alpha;
	
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
			beziers[i] = Bezier.getFromHermite(waypoints[i].asVector(), waypoints[i + 1].asVector(),
					new Vec2D(Math.cos(waypoints[i].getHeading()) * alpha, Math.sin(waypoints[i].getHeading()) * alpha),
					new Vec2D(Math.cos(waypoints[i + 1].getHeading()) * alpha, Math.sin(waypoints[i + 1].getHeading()) * alpha));
		}
		
		this.waypoints = waypoints;
		this.alpha = alpha;
		
		resetIntegration();
		resetWheelIntegration();
	}
	public BezierPath(Waypoint[] waypoints, double alpha, double baseRadius) {
		this(waypoints, alpha);
		this.baseRadius = baseRadius;
	}
	
	/**
	 * Sets the base plate radius (distance from the center of the robot to the wheels) of the robot following
	 * this path. This value is used to compute the result from {@link BezierPath#wheelsAt(double) wheelsAt()}.
	 * @param b - The new base radius
	 */
	public void setBaseRadius(double b) {
		baseRadius = b;
		//Since a changed base radius means the wheels will start at different locations, the integration must
		//be reset to avoid bugs
		resetWheelIntegration();
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
		if(t >= 1) {
			return beziers[beziers.length - 1].at(1);
		}
		
		t *= beziers.length;
		return beziers[(int) Math.floor(t)].at(t % 1.0);
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
		if(t >= 1) {
			return beziers[beziers.length - 1].derivAt(1);
		}
		
		t *= beziers.length;
		return beziers[(int) Math.floor(t)].derivAt(t % 1.0);
	}
	/**
	 * Returns the second derivative at a specified time in the path.
	 * @param t - A positive real number in the range 0 to 1
	 * @return The second derivative of the path at the specified time
	 */
	public Vec2D secondDerivAt(double t) {
		if(t >= 1) {
			return beziers[beziers.length - 1].secondDerivAt(1);
		}
		
		t *= beziers.length;
		return beziers[(int) Math.floor(t)].secondDerivAt(t % 1.0);
	}
	
	/**
	 * Integrates length of the path.
	 * @param dt - dt
	 * @return The length of the path after a time increment of dt
	 */
	public double integrateLen(double dt) {
		iCurrentTime += dt;
		if(iCurrentTime > 1) {
			return iCurrentDist;
		}
		Vec2D currentPoint = at(iCurrentTime);
		iCurrentDist += iLastPos.distTo(currentPoint);
		iLastPos = currentPoint;
		return iCurrentDist;
	}
	/**
	 * Resets the integration of the length of the path.
	 */
	public void resetIntegration() {
		iCurrentTime = 0;
		iLastPos = at(0);
		iCurrentDist = 0;
	}
	/**
	 * Retrieves the length of the path integrated by {@link BezierPath#integrateLen(double) integrateLen()}.
	 * @return The length of the path integrated by {@link BezierPath#integrateLen(double) integrateLen()}
	 */
	public double getIntegratedLen() {
		return iCurrentDist;
	}
	
	/**
	 * Integrates length of the path of the wheels.
	 * @param dt - dt
	 * @return The length of the path after a time increment of dt
	 */
	public double[] integrateWheelLens(double dt) {
		iwCurrentTime += dt;
		if(iwCurrentTime > 1)
			return iwCurrentDists;
		Vec2D[] iwCurrentPos = wheelsAt(iwCurrentTime);
		iwCurrentDists[0] += iwLastPos[0].distTo(iwCurrentPos[0]);
		iwCurrentDists[1] += iwLastPos[1].distTo(iwCurrentPos[1]);
		iwLastPos = iwCurrentPos;
		return iwCurrentDists;
	}
	/**
	 * Resets the integration of the length of the path of the wheels.
	 */
	public void resetWheelIntegration() {
		iwCurrentTime = 0;
		iwLastPos = wheelsAt(0);
		iwCurrentDists = new double[2];
	}
	/**
	 * Retrieves the length of the path of the wheels integrated by {@link BezierPath#integrateWheelLens(double) integrateWheelLens()}.
	 * @return The length of the path of the wheels integrated by {@link BezierPath#integrateWheelLens(double) integrateWheelLens()}
	 */
	public double[] getIntegratedWheelLens() {
		//Return a clone so we can't accidentally modify it outside the class
		return iwCurrentDists.clone();
	}
	
	/**
	 * Retrieves the waypoints used to construct this path.
	 * @return The waypoints that were used to construct this path
	 */
	public Waypoint[] getWaypoints() {
		return waypoints;
	}
	/**
	 * Retrieves the alpha (smoothness constant) of this path.
	 * @return The alpha of this path
	 */
	public double getAlpha() {
		return alpha;
	}
}
