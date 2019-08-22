package com.arctos6135.robotpathfinder.util;

import java.util.Objects;

/**
 * A class that holds a pair of values.
 *
 * @author Tyler Tian
 * @param <T> The type of the first element
 * @param <U> The type of the second element
 * @since 3.0.0
 */
public class Pair<T, U> {

	protected T first;
	protected U second;

	@Override
	public boolean equals(Object o) {
		if (o == this)
			return true;
		if (!(o instanceof Pair)) {
			return false;
		}
		Pair<?, ?> pair = (Pair<?, ?>) o;
		return Objects.equals(first, pair.first) && Objects.equals(second, pair.second);
	}

	@Override
	public int hashCode() {
		return Objects.hash(first, second);
	}

	@Override
	public String toString() {
		return "{" + " first='" + getFirst() + "'" + ", second='" + getSecond() + "'" + "}";
	}

	/**
	 * Creates a new {@link Pair} object with both elements {@code null}.
	 */
	public Pair() {
		first = null;
		second = null;
	}

	/**
	 * Creates a new {@link Pair} object with the specified elements.
	 * 
	 * @param e1 The first element
	 * @param e2 The second element
	 */
	public Pair(T e1, U e2) {
		first = e1;
		second = e2;
	}

	/**
	 * Retrieves the first element of the pair.
	 * 
	 * @return The first element
	 */
	public T getFirst() {
		return first;
	}

	/**
	 * Retrieves the second element of the pair.
	 * 
	 * @return The second element
	 */
	public U getSecond() {
		return second;
	}

	/**
	 * Sets the first element of the pair.
	 * 
	 * @param e1 The new value for the first element
	 */
	public void setFirst(T e1) {
		first = e1;
	}

	/**
	 * Sets the second element of the pair.
	 * 
	 * @param e2 The new value for the second element
	 */
	public void setSecond(U e2) {
		second = e2;
	}
}
