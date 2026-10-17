package btools.router;

/**
 * Raised when a leg finds no track while the no-U-turn guard ({@code continueStraight}) rejected at
 * least one of its first steps. Extends {@link IllegalArgumentException} and keeps the historic message
 * ("no track found at pass=N (without a U-turn at ...)") so callers that read
 * {@link RoutingEngine#getErrorMessage()} are unaffected; typed callers read
 * {@link RoutingEngine#getReverseGuardBlockedWaypointIndex()} instead of parsing text.
 */
public class ReverseGuardBlockedException extends IllegalArgumentException {
  /** Index of the waypoint the leg could not leave, or -1 where the leg's position in the request is not known. */
  private final int waypointIndex;

  public ReverseGuardBlockedException(String message) {
    this(-1, message);
  }

  /**
   * @param waypointIndex index of the waypoint the leg starts at, in the list the engine was given
   * @param message       human-readable description
   */
  public ReverseGuardBlockedException(int waypointIndex, String message) {
    super(message);
    this.waypointIndex = waypointIndex;
  }

  public int getWaypointIndex() {
    return waypointIndex;
  }
}
