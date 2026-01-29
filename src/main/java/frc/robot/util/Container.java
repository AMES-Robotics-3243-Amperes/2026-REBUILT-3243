package frc.robot.util;

/*
 * A common issue with commands is that a variable needs to be defined in the enclosing scope
 * but used in a periodic callback and thus must be final. This is used to avoid that issue
 * by wrapping the object in a final Container object, leaving the inner object mutable.
 */
public class Container<T> {
  public T inner;

  public Container(T inner) {
    this.inner = inner;
  }
}
