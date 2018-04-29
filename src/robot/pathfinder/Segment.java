package robot.pathfinder;

public class Segment {
	
	public double maxVelo;
	public double r;
	public double start, end;
	
	public Segment(double start, double end) {
		this.start = start;
		this.end = end;
	}
	
	public void setMaxVelo(double maxVelo) {
		this.maxVelo = maxVelo;
	}
	public double getMaxVelo() {
		return maxVelo;
	}
	public void setR(double r) {
		this.r = r;
	}
	public double getR() {
		return r;
	}
	
}
