package robot.pathfinder.math;

/**
 * A class containing a collection of static math utility methods.
 * @author Tyler Tian
 *
 */
public class MathUtils {
	
	/**
	 * Swaps two rows of a matrix/two arrays.
	 * @param a - The first row
	 * @param b - The second row
	 */
	public static void rowSwap(double[] a, double[] b) {
		double[] temp = new double[a.length];
		for(int i = 0; i < temp.length; i ++) {
			
			temp[i] = a[i];
			a[i] = b[i];
			b[i] = temp[i];
		}
	}
	/**
	 * Multiplies a row of a matrix/an array by a scalar
	 * @param a - The row to work with
	 * @param b - A scalar to multiply by
	 */
	public static void rowMultiply(double[] a, double b) {
		for(int i = 0; i < a.length; i ++) {
			a[i] *= b;
		}
	}
	/**
	 * Multiplies a row of a matrix/an array by a scalar, and return the result. <br>
	 * <br>
	 * Unlike in {@link MathUtils#rowMultiply(double[], double) rowMutiply()}, the original row/array is not modified.
	 * @param a - The row to work with
	 * @param b - A scalar to multiply by
	 * @return The row with each term multiplied by the scalar
	 */
	public static double[] rowMultiply2(double[] a, double b) {
		double[] c = new double[a.length];
		for(int i = 0; i < c.length; i ++) {
			c[i] = a[i] * b;
		}
		return c;
	}
	/**
	 * Adds each term of a row of a matrix/an array to another row/array.
	 * @param a - The row to add to
	 * @param b - The row to add
	 */
	public static void rowAdd(double[] a, double[] b) {
		for(int i = 0; i < a.length; i ++) {
			a[i] += b[i];
		}
	}
	
	static int lcIndex(double[] a) {
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
	 * @param mat - The matrix representing the system of equations
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
	 * @param a - The squared term coefficient
	 * @param b - The linear term coefficient
	 * @param c - The constant
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
	 * @param a - The squared term coefficient
	 * @param b - The linear term coefficient
	 * @param c - The constant
	 * @param minUnit - The rounding limit for {@code b^2-4ac}
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
	 * @param a - The squared term coefficient
	 * @param b - The linear term coefficient
	 * @param c - The constant
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
	 * @param a - The squared term coefficient
	 * @param b - The linear term coefficient
	 * @param c - The constant
	 * @param minUnit - The rounding limit for {@code b^2-4ac} - For more information see {@link MathUtils#findQuadraticRoots(double, double, double, double) findQuadraticRoots()}.
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
}
