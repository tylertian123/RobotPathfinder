package robot.pathfinder.math;

import robot.pathfinder.core.Waypoint;

/**
 * A class that represents a vector in 2D space.
 * @author Tyler Tian
 *
 */
public class Vec2D {
	
	private double x, y;
	
	/**
	 * The vector [0, 0]; the additive identity.
	 */
	public static final Vec2D zero = new Vec2D(0, 0);
	
	@Override
	public String toString() {
		return "[" + x + ", " + y + "]";
	}
	
	/**
	 * Constructs a new vector with the specified x and y values.
	 * @param x The X value of the vector
	 * @param y The Y value of the vector
	 */
	public Vec2D(double x, double y) {
		this.x = x;
		this.y = y;
	}
	/**
	 * Constructs a new vector with the location of the waypoint. The heading is ignored.
	 * @param w The waypoint whose location will be taken
	 */
	public Vec2D(Waypoint w) {
		this.x = w.getX();
		this.y = w.getY();
	}
	
	/**
	 * Retrieves the X value of this vector.
	 * @return The X value of this vector
	 */
	public double getX() {
		return x;
	}
	/**
	 * Retrieves the Y value of this vector.
	 * @return The Y value of this vector
	 */
	public double getY() {
		return y;
	}
	
	/**
	 * Adds 2 or more vectors and returns the result.
	 * @param vecs The vectors to add
	 * @return The result from adding the vectors
	 */
	public static Vec2D addVecs(Vec2D... vecs) {
		Vec2D v = new Vec2D(0, 0);
		for(Vec2D vec : vecs) {
			v.x += vec.x;
			v.y += vec.y;
		}
		return v;
	}
	/**
	 * Calculates the distance between 2 vectors.
	 * @param a The first vector
	 * @param b The second vector
	 * @return The distance between the 2 vectors
	 */
	public static double dist(Vec2D a, Vec2D b) {
		return Math.hypot(a.x - b.x, a.y - b.y);
	}
	/**
	 * Linearly interpolates between two vectors.
	 * @param a The first vector
	 * @param b The second vector
	 * @param f The fraction of the way from the first to the second
	 * @return The result of the lerp
	 */
	public static Vec2D lerp(Vec2D a, Vec2D b, double f) {
		return new Vec2D(MathUtils.lerp(a.x, b.x, f), MathUtils.lerp(a.y, b.y, f));
	}
	/**
	 * Returns the normalized copy of the vector.
	 * @param vec The vector to normalize
	 * @return The normalized vector
	 */
	public static Vec2D normalized(Vec2D vec) {
		double mag = vec.magnitude();
		
		return new Vec2D(vec.x / mag, vec.y / mag);
	}
	
	/**
	 * Multiplies this vector by a scalar and returns the result. Note that this vector is not modified in the process.
	 * @param scalar The scalar to multiply by
	 * @return The result of the multiplication
	 */
	public Vec2D multiply(double scalar) {
		return new Vec2D(x * scalar, y * scalar);
	}
	/**
	 * Adds this vector to another vector and returns the result. Note that this vector is not modified in the process.
	 * @param vec The vector to add to
	 * @return The result of the addition
	 */
	public Vec2D add(Vec2D vec) {
		return new Vec2D(x + vec.x, y + vec.y);
	}
	/**
	 * Subtracts a vector from this vector and returns the result. Note that this vector is not modified int the process.
	 * @param vec The vector to subtract
	 * @return The result of the subtraction
	 */
	public Vec2D subtract(Vec2D vec) {
		return new Vec2D(x - vec.x, y - vec.y);
	}
	/**
	 * Calculates the distance between this vector and another vector.
	 * @param vec The other vector
	 * @return The distance between the two
	 */
	public double distTo(Vec2D vec) {
		return Vec2D.dist(this, vec);
	}
	/**
	 * Calculates the magnitude of this vector.
	 * @return The magnitude of this vector
	 */
	public double magnitude() {
		return Math.hypot(x, y);
	}
	/**
	 * Normalizes this vector.
	 */
	public void normalize() {
		double mag = magnitude();
		x /= mag;
		y /= mag;
	}
	/**
	 * Calculates the dot product of this vector and the specified vector.
	 * @param vec The vector to take the dot product with
	 * @return The dot product
	 */
	public double dot(Vec2D vec) {
		return x * vec.x + y * vec.y;
	}
	/**
	 * Projects this vector onto another. Equivalent to <code>proj</code><i><sub>v</sub>u</i>, where
	 * <i>u</i> is this vector and <i>v</i> is the vector provided.
	 * @param vec The vector to project onto
	 * @return The projection of this vector onto the provided vector
	 */
	public Vec2D proj(Vec2D vec) {
		double mag = dot(vec) / vec.magnitude();
		return Vec2D.normalized(vec).multiply(mag);
	}
	/**
	 * Reflects this vector across the line defined by another. 
	 * @param vec The vector representing the line to reflect across
	 * @return The result of the reflection
	 */
	public Vec2D reflect(Vec2D vec) {
		// Nice geometrical property
		return proj(vec).multiply(2).subtract(this);
	}
}
