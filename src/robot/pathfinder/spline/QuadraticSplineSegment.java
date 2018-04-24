package robot.pathfinder.spline;

public class QuadraticSplineSegment {
	protected double a, b, c;
	protected double start, end;
	
	public QuadraticSplineSegment(double a, double b, double c, double start, double end) {
		this.a = a;
		this.b = b;
		this.c = c;
		this.start = start;
		this.end = end;
	}
	public QuadraticSplineSegment(double[] coefficients, double start, double end) {
		for(double d : coefficients)
			if(Double.isNaN(d))
				throw new IllegalArgumentException("A parameter is NaN");
		this.a = coefficients[0];
		this.b = coefficients[1];
		this.c = coefficients[2];
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
		return a * Math.pow(t, 2) + b * t + c;
	}
	
	public double derivAt(double t) {
		if(!includes(t))
			throw new IllegalArgumentException("Out of range");
		return 2 * a * t + b;
	}
	
	public double secondDerivAt(double t) {
		if(!includes(t))
			throw new IllegalArgumentException("Out of range");
		return 2 * a;
	}
}
