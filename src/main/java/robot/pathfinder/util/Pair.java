package robot.pathfinder.util;

/**
 * A class that holds a pair of related values.
 * @author Tyler Tian
 *
 * @param <T> The type of the first element
 * @param <U> The type of the second element
 */
public class Pair<T, U> {
	
	private T elem1;
	private U elem2;
	
	/**
	 * Creates a new {@link Pair} object with both elements {@code null}.
	 */
	public Pair() {
		elem1 = null;
		elem2 = null;
	}
	/**
	 * Creates a new {@link Pair} object with the specified elements.
	 * @param e1 The first element
	 * @param e2 The second element
	 */
	public Pair(T e1, U e2) {
		elem1 = e1;
		elem2 = e2;
	}
	
	/**
	 * Retrieves the first element of the pair.
	 * @return The first element
	 */
	public T getElem1() {
		return elem1;
	}
	/**
	 * Retrieves the second element of the pair.
	 * @return The second element
	 */
	public U getElem2() {
		return elem2;
	}
	/**
	 * Sets the first element of the pair.
	 * @param e1 The new value for the first element
	 */
	public void setElem1(T e1) {
		elem1 = e1;
	}
	/**
	 * Sets the second element of the pair.
	 * @param e2 The new value for the second element
	 */
	public void setElem2(U e2) {
		elem2 = e2;
	}
}
