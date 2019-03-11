package com.arctos6135.robotpathfinder.util;

/**
 * A class that holds a pair of values.
 * @author Tyler Tian
 *
 * @param <T> The type of the first element
 * @param <U> The type of the second element
 */
public class Pair<T, U> {
	
	protected T first;
	protected U second;
	
	/**
	 * Creates a new {@link Pair} object with both elements {@code null}.
	 */
	public Pair() {
		first = null;
		second = null;
	}
	/**
	 * Creates a new {@link Pair} object with the specified elements.
	 * @param e1 The first element
	 * @param e2 The second element
	 */
	public Pair(T e1, U e2) {
		first = e1;
		second = e2;
	}
	
	/**
	 * Retrieves the first element of the pair.
	 * @return The first element
	 */
	public T getFirst() {
		return first;
	}
	/**
	 * Retrieves the second element of the pair.
	 * @return The second element
	 */
	public U getSecond() {
		return second;
	}
	/**
	 * Sets the first element of the pair.
	 * @param e1 The new value for the first element
	 */
	public void setFirst(T e1) {
		first = e1;
	}
	/**
	 * Sets the second element of the pair.
	 * @param e2 The new value for the second element
	 */
	public void setSecond(U e2) {
		second = e2;
	}
}
