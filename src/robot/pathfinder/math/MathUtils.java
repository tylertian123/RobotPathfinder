package robot.pathfinder.math;

/**
 * A class containing a collection of static math utility methods.
 * @author Tyler Tian
 *
 */
public class MathUtils {
	
	private MathUtils() {}
	
	private static void rowSwap(double[] a, double[] b) {
		double[] temp = new double[a.length];
		for(int i = 0; i < temp.length; i ++) {
			
			temp[i] = a[i];
			a[i] = b[i];
			b[i] = temp[i];
		}
	}
	private static void rowMultiply(double[] a, double b) {
		for(int i = 0; i < a.length; i ++) {
			a[i] *= b;
		}
	}
	private static double[] rowMultiply2(double[] a, double b) {
		double[] c = new double[a.length];
		for(int i = 0; i < c.length; i ++) {
			c[i] = a[i] * b;
		}
		return c;
	}
	private static void rowAdd(double[] a, double[] b) {
		for(int i = 0; i < a.length; i ++) {
			a[i] += b[i];
		}
	}
	private static int lcIndex(double[] a) {
		//Finds the leading coefficient's index in a row.
		//A larger index means the leading coefficient is closer to the left
		for(int i = 0; i < a.length - 1; i ++) {
			if(a[i] != 0)
				return a.length - 1 - i;
		}
		return 0;
	}
	
	/**
	 * Solves a system of linear equations represented in a matrix, using the Gauss-Jordan elimination method.<br>
	 * <br>
	 * For more information see <a href="https://en.wikipedia.org/wiki/Gaussian_elimination">this Wikipedia article</a>.
	 * @param mat The matrix representing the system of equations
	 * @return The value of each unknown in order
	 */
	public static double[] solve(double[][] mat) {
		if(mat.length + 1 != mat[0].length)
			throw new IllegalArgumentException();
		//Make sure the rows with the leftmost leading coefficients are placed first
		int[] lcIndexes = new int[mat.length];
		for(int i = 0; i < lcIndexes.length; i ++) {
			lcIndexes[i] = lcIndex(mat[i]);
		}
		//Bubble sort
		boolean sorted;
		do {
			sorted = true;
			for(int i = 0; i < mat.length - 1; i ++) {
				if(lcIndexes[i] < lcIndexes[i + 1]) {
					sorted = false;
					rowSwap(mat[i], mat[i + 1]);
					int buf = lcIndexes[i];
					lcIndexes[i] = lcIndexes[i + 1];
					lcIndexes[i + 1] = buf;
				}
			}
		} while(!sorted);
		//Make sure pivots are always non-zero
		boolean pass;
		do {
			pass = true;
			for(int i = 0; i < mat.length; i ++) {
				if(mat[i][i] == 0) {
					pass = false;
					boolean found = false;
					for(int j = 0; j < mat.length; j ++) {
						if(mat[j][i] != 0 && lcIndexes[j] == lcIndexes[i]) {
							rowSwap(mat[i], mat[j]);
							int buf = lcIndexes[i];
							lcIndexes[i] = lcIndexes[j];
							lcIndexes[j] = buf;
							found = true;
						}
					}
					if(!found)
						throw new IllegalArgumentException("Infinitely many solutions to this equation");
				}
			}
		} while(!pass);
		//Transform to row echelon form
		int pivot = 0;
		while(pivot < mat.length - 1) {
			for(int i = pivot + 1; i < mat.length; i ++) {
				if(mat[i][pivot] != 0) {
					double ratio = -mat[i][pivot] / mat[pivot][pivot];
					rowAdd(mat[i], rowMultiply2(mat[pivot], ratio));
				}
			}
			pivot ++;
		}
		//Transform to reduced row echelon form
		for(int i = 0; i < mat.length; i ++) {
			if(mat[i][i] != 1) {
				rowMultiply(mat[i], 1.0 / mat[i][i]);
			}
		}
		//Reduce
		pivot = mat.length - 1;
		while(pivot > 0) {
			for(int i = 0; i < pivot; i ++) {
				if(mat[i][pivot] != 0) {
					rowAdd(mat[i], rowMultiply2(mat[pivot], -mat[i][pivot]));
				}
			}
			pivot --;
		}
		//Return the solution
		double[] solution = new double[mat.length];
		for(int i = 0; i < solution.length; i ++) {
			solution[i] = mat[i][mat.length];
		}
		return solution;
	}
	
	/**
	 * Finds the roots of a quadratic equation in standard form.<br>
	 * <br>
	 * If no real roots exist, the resulting array will contain {@code [NaN, NaN]}; if the two roots
	 * are the same, the two elements of the array will be the same number.
	 * @param a The squared term coefficient
	 * @param b The linear term coefficient
	 * @param c The constant
	 * @return The roots of the equation
	 */
	public static double[] findQuadraticRoots(double a, double b, double c) {
		return findQuadraticRoots(a, b, c, 0);
	}
	/**
	 * Finds the roots of a quadratic equation in standard form.<br>
	 * <br>
	 * If no real roots exist, the resulting array will contain {@code [NaN, NaN]}; if the two roots
	 * are the same, the two elements of the array will be the same number.<br>
	 * <br>
	 * If the absolute value of {@code b^2-4ac} is less than or equal to the rounding limit, it will
	 * be rounded down to 0; this can help prevent the situation in which perfectly fine equations become
	 * unsolvable due to rounding issues.
	 * @param a The squared term coefficient
	 * @param b The linear term coefficient
	 * @param c The constant
	 * @param minUnit The rounding limit for {@code b^2-4ac}
	 * @return The roots of the equation
	 */
	public static double[] findQuadraticRoots(double a, double b, double c, double minUnit) {
		//Special case
		if(a == 0) {
			return new double[] {
					-c / b,
					-c / b
			};
		}
		double d = b * b - 4 * a * c;
		if(Math.abs(d) <= minUnit)
			d = 0;
		double r = Math.sqrt(d);
		return new double[] {
				(-b + r) / (2 * a),
				(-b - r) / (2 * a),
		};
	}
	/**
	 * Finds the roots of a quadratic equation using {@link MathUtils#findQuadraticRoots(double, double, double) findQuadraticRoots()},
	 * and returns the positive root. If both roots are negative, this method will return {@code NaN}.
	 * @param a The squared term coefficient
	 * @param b The linear term coefficient
	 * @param c The constant
	 * @return The positive root of the quadratic equation, or if both roots are negative, {@code NaN}.
	 */
	public static double findPositiveQuadraticRoot(double a, double b, double c) {
		double[] roots = findQuadraticRoots(a, b, c);
		if(roots[0] >= 0)
			return roots[0];
		else if(roots[1] >= 0)
			return roots[1];
		else
			return Double.NaN;
	}
	/**
	 * Finds the roots of a quadratic equation using {@link MathUtils#findQuadraticRoots(double, double, double, double) findQuadraticRoots()},
	 * and returns the positive root. If both roots are negative, this method will return {@code NaN}.
	 * @param a The squared term coefficient
	 * @param b The linear term coefficient
	 * @param c The constant
	 * @param minUnit - The rounding limit for {@code b^2-4ac} For more information see {@link MathUtils#findQuadraticRoots(double, double, double, double) findQuadraticRoots()}.
	 * @return The positive root of the quadratic equation, or if both roots are negative, {@code NaN}.
	 */
	public static double findPositiveQuadraticRoot(double a, double b, double c, double minUnit) {
		double[] roots = findQuadraticRoots(a, b, c, minUnit);
		if(roots[0] >= 0)
			return roots[0];
		else if(roots[1] >= 0)
			return roots[1];
		else
			return Double.NaN;
	}
	
	/**
	 * Finds the discriminant of a cubic polynomial.<br>
	 * <br>
	 * If this value is positive, then the polynomial has 3 distinct real roots.<br>
	 * If this value is zero, then the polynomial has a multiple root and all 3 roots are real.<br>
	 * If this value is negative, then the polynomial has one real root, and 2 complex roots.
	 * @param a The cubed term coefficient
	 * @param b The squared term coefficient
	 * @param c The linear term coefficient
	 * @param d The constant term
	 * @return The discriminant of the polynomial
	 */
	public static double cubicDiscriminant(double a, double b, double c, double d) {
		//Formula can be found on Wikipedia: https://en.wikipedia.org/wiki/Cubic_function#General_formula
		return 18 * a * b * c * d - 4 * Math.pow(b, 3) * d + Math.pow(b, 2) * Math.pow(c, 2) - 4 * a * Math.pow(c, 3) - 27 * Math.pow(a, 2) * Math.pow(d, 2);
	}
	/**
	 * Finds the real root of a cubic polynomial. This method assumes that the discriminant of the polynomial is negative, that is,
	 * the polynomial has only one real root and 2 complex roots.<br>
	 * <br>
	 * If the discriminant is greater than 0, this method will return {@code NaN}.
	 * @param a The cubed term coefficient
	 * @param b The squared term coefficient
	 * @param c The linear term coefficient
	 * @param d The constant term
	 * @return The real root
	 */
	public static double realCubicRoot(double a, double b, double c, double d) {
		if(a == 0.0) {
			return findPositiveQuadraticRoot(b, c, d);
		}
		
		//Formula can be found on Wikipedia: https://en.wikipedia.org/wiki/Cubic_function#General_formula
		double d0 = b * b - 3 * a * c;
		double d1 = 2 * Math.pow(b, 3) - 9 * a * b * c + 27 * Math.pow(a, 2) * d;
		double C;
		//"In addition either sign in front of the square root may be chosen unless d0 = 0
		//in which case the sign must be chosen so that the two terms inside the cube root do not cancel."
		if(d0 != 0) {
			C = Math.cbrt((d1 - Math.sqrt(Math.pow(d1, 2) - 4 * Math.pow(d0, 3))) / 2);
		}
		else {
			if(d1 >= 0) {
				//If d1 is greater than 0, then we must add, because the result of the square root is always going to be positive
				C = Math.cbrt((d1 + Math.sqrt(Math.pow(d1, 2) - 4 * Math.pow(d0, 3))) / 2);
			}
			else {
				//If d1 is less than 0, we must subtract
				C = Math.cbrt((d1 - Math.sqrt(Math.pow(d1, 2) - 4 * Math.pow(d0, 3))) / 2);
			}
		}
		
		return -1 / (3 * a) * (b + C + d0 / C);
	}
	
	/**
	 * Calculates the curvature, given the derivatives and the second derivatives at the desired point.
	 * @param xDeriv The first derivative of x, with respect to t (dx/dt).
	 * @param xSecondDeriv The second derivative of x, with respect to t (d^2x/dt^2).
	 * @param yDeriv The first derivative of y, with respect to t (dy/dt).
	 * @param ySecondDeriv The second derivative of y, with respect to t (d^2y/dt^2).
	 * @return The curvature at the point
	 */
	public static double curvature(double xDeriv, double xSecondDeriv, double yDeriv, double ySecondDeriv) {
		return (xDeriv * ySecondDeriv - yDeriv * xSecondDeriv) /
				Math.pow(xDeriv * xDeriv + yDeriv * yDeriv, 3.0/2.0);
	}
}
