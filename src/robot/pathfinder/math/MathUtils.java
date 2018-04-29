package robot.pathfinder.math;

public class MathUtils {
	
	public static void rowSwap(double[] a, double[] b) {
		double[] temp = new double[a.length];
		for(int i = 0; i < temp.length; i ++) {
			
			temp[i] = a[i];
			a[i] = b[i];
			b[i] = temp[i];
		}
	}
	public static void rowMultiply(double[] a, double b) {
		for(int i = 0; i < a.length; i ++) {
			a[i] *= b;
		}
	}
	public static double[] rowMultiply2(double[] a, double b) {
		double[] c = new double[a.length];
		for(int i = 0; i < c.length; i ++) {
			c[i] = a[i] * b;
		}
		return c;
	}
	public static void rowAdd(double[] a, double[] b) {
		for(int i = 0; i < a.length; i ++) {
			a[i] += b[i];
		}
	}
	
	public static int lcIndex(double[] a) {
		for(int i = 0; i < a.length - 1; i ++) {
			if(a[i] != 0)
				return a.length - 1 - i;
		}
		return 0;
	}
	
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
	
	public static Circle getCircleFromPoints(Vec2D a, Vec2D b, Vec2D c) {
		double[] center = MathUtils.solve(new double[][] {
			{ 2 * (b.getX() - a.getX()), 2 * (b.getY() - a.getY()), b.getX() * b.getX() - a.getX() * a.getX() + b.getY() * b.getY() - a.getY() * a.getY() },
			{ 2 * (c.getX() - b.getX()), 2 * (c.getY() - b.getY()), c.getX() * c.getX() - b.getX() * b.getX() + c.getY() * c.getY() - b.getY() * b.getY() },
		});
		Vec2D centerVec = new Vec2D(center[0], center[1]);
		return new Circle(centerVec, centerVec.distTo(a));
	}
	
	public static double[] findQuadraticRoots(double a, double b, double c) {
		double r = Math.sqrt(b * b - 4 * a * c);
		return new double[] {
				(-b + r) / (2 * a),
				(-b - r) / (2 * a),
		};
	}
	public static double findPositiveQuadraticRoot(double a, double b, double c) {
		double[] roots = findQuadraticRoots(a, b, c);
		if(roots[0] >= 0)
			return roots[0];
		else if(roots[1] >= 0)
			return roots[1];
		else
			return Double.NaN;
	}
}
