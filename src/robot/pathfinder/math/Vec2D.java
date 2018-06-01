package robot.pathfinder.math;

import robot.pathfinder.Waypoint;

/**
 * A class that represents a vector in 2D space.
 * @author Tyler Tian
 *
 */
public class Vec2D {
	
	double x, y;
	
	/**
	 * Constructs a new vector with the specified x and y values.
	 * @param x - The X value of the vector
	 * @param y - The Y value of the vector
	 */
	public Vec2D(double x, double y) {
		this.x = x;
		this.y = y;
	}
	/**
	 * Constructs a new vector with the location of the waypoint. The heading is ignored.
	 * @param w - The waypoint whose location will be taken
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
	 * @param vecs - The vectors to add
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
	 * Calculates the distance between 2 vectors
	 * @param a - The first vector
	 * @param b - The second vector
	 * @return The distance between the 2 vectors
	 */
	
	public static double dist(Vec2D a, Vec2D b) {
		return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
	}
	/**
	 * Multiplies this vector by a scalar and returns the result. Note that this vector is not modified in the process.
	 * @param scalar - The scalar to multiply by
	 * @return The result of the multiplication
	 */
	public Vec2D multiply(double scalar) {
		return new Vec2D(x * scalar, y * scalar);
	}
	/**
	 * Adds this vector to another vector and returns the result. Note that this vector is not modified in the process.
	 * @param vec - The vector to add to
	 * @return The result of the addition
	 */
	public Vec2D add(Vec2D vec) {
		return new Vec2D(x + vec.x, y + vec.y);
	}
	/**
	 * Subtracts a vector from this vector and returns the result. Note that this vector is not modified int the process.
	 * @param vec - The vector to subtract
	 * @return The result of the subtraction
	 */
	public Vec2D subtract(Vec2D vec) {
		return new Vec2D(x - vec.x, y - vec.y);
	}
	/**
	 * Calculates the distance between this vector and another vector.
	 * @param vec - The other vector
	 * @return The distance between the two
	 */
	public double distTo(Vec2D vec) {
		return Vec2D.dist(this, vec);
	}
	
	/**
	 * Calculates the coordinates of the vector given relative to this vector.
	 * @param vec - The vector whose coordinates will be taken
	 * @return The relative coordinates of the vector to this vector
	 */
	public Vec2D relative(Vec2D vec) {
		return new Vec2D(vec.x - this.x, vec.y - this.y);
	}
}
