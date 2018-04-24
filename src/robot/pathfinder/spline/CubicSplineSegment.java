package robot.pathfinder.spline;

public class CubicSplineSegment {
	protected double a, b, c, d;
	protected double start, end;
	
	public CubicSplineSegment(double a, double b, double c, double d, double start, double end) {
		this.a = a;
		this.b = b;
		this.c = c;
		this.d = d;
		this.start = start;
		this.end = end;
	}
	public CubicSplineSegment(double[] coefficients, double start, double end) {
		for(double d : coefficients)
			if(Double.isNaN(d))
				throw new IllegalArgumentException("A parameter is NaN");
		this.a = coefficients[0];
		this.b = coefficients[1];
		this.c = coefficients[2];
		this.d = coefficients[3];
		this.start = start;
		this.end = end;
	}
	
	public double getStart() {
		return start;
	}
	public double getEnd() {
		return end;
	}
	
	public boolean includes(double t) {
		return t >= start && t <= end;
	}
	
	public double at(double t) {
		if(!includes(t))
			throw new IllegalArgumentException("Out of range");
		return a * Math.pow(t, 3) + b * Math.pow(t, 2) + c * t + d;
	}
	
	public double derivAt(double t) {
		if(!includes(t))
			throw new IllegalArgumentException("Out of range");
		return 3 * a * Math.pow(t, 2) + 2 * b * t + c;
	}
	
	public double secondDerivAt(double t) {
		if(!includes(t))
			throw new IllegalArgumentException("Out of range");
		return 6 * a * t + 2 * b;
	}
}
