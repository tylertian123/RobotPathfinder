package robot.pathfinder;

public class Segment {
	public double start, end;
	public double maxVelo;
	
	public Segment(double start, double end) {
		this(start, end, 0);
	}
	public Segment(double start, double end, double maxVelo) {
		this.start = start;
		this.end = end;
		this.maxVelo = maxVelo;
	}
	
	public void setMaxVelo(double maxVelo) {
		this.maxVelo = maxVelo;
	}
	public double getMaxVelo() {
		return maxVelo;
	}
	public double getMid() {
		return (start + end) / 2;
	}
}
