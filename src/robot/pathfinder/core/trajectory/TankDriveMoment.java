package robot.pathfinder.core.trajectory;

public class TankDriveMoment implements Cloneable {
	
	double ld, lv, la, rd, rv, ra;
	double t;
	double heading;
	
	public TankDriveMoment() {
		ld = lv = la = rd = rv = ra = t = heading = 0;
	}
	
	@Override
	public TankDriveMoment clone() {
		return new TankDriveMoment(ld, rd, lv, rv, la, ra, heading, t);
	}
	
	public TankDriveMoment(double leftPos, double rightPos, double leftVel, double rightVel, double leftAcl, double rightAcl, double heading) {
		ld = leftPos;
		rd = rightPos;
		lv = leftVel;
		rv = rightVel;
		la = leftAcl;
		ra = rightAcl;
		this.heading = heading;
	}
	
	public TankDriveMoment(double leftPos, double rightPos, double leftVel, double rightVel, double leftAcl, double rightAcl, double heading, double time) {
		this(leftPos, rightPos, leftVel, rightVel, leftAcl, rightAcl, heading);
		t = time;
	}

	public double getLeftPosition() {
		return ld;
	}

	public void setLeftPosition(double ld) {
		this.ld = ld;
	}

	public double getLeftVelocity() {
		return lv;
	}

	public void setLeftVelocity(double lv) {
		this.lv = lv;
	}

	public double getLeftAcceleration() {
		return la;
	}

	public void setLeftAcceleration(double la) {
		this.la = la;
	}

	public double getRightPosition() {
		return rd;
	}

	public void setRightPosition(double rd) {
		this.rd = rd;
	}

	public double getRightVelocity() {
		return rv;
	}

	public void setRightVelocity(double rv) {
		this.rv = rv;
	}

	public double getRightAcceleration() {
		return ra;
	}

	public void setRightAcceleration(double ra) {
		this.ra = ra;
	}

	public double getTime() {
		return t;
	}

	public void setTime(double t) {
		this.t = t;
	}

	public double getHeading() {
		return heading;
	}

	public void setHeading(double heading) {
		this.heading = heading;
	}
	
	public BasicMoment leftComponent() {
		return new BasicMoment(ld, lv, la, heading, t);
	}
	public BasicMoment rightComponent() {
		return new BasicMoment(rd, rv, ra, heading, t);
	}
	
	public static TankDriveMoment fromComponents(BasicMoment left, BasicMoment right) {
		return new TankDriveMoment(left.getPosition(), right.getPosition(), left.getVelocity(), right.getVelocity(),
				left.getAcceleration(), right.getAcceleration(), left.getHeading(), left.getTime());
	}
}
