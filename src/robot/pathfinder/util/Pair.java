package robot.pathfinder.util;

public class Pair<T, U> {
	
	private T elem1;
	private U elem2;
	
	public Pair(T e1, U e2) {
		elem1 = e1;
		elem2 = e2;
	}
	
	public T getElem1() {
		return elem1;
	}
	public U getElem2() {
		return elem2;
	}
	
	public void setElem1(T e1) {
		elem1 = e1;
	}
	public void setElem2(U e2) {
		elem2 = e2;
	}
}
