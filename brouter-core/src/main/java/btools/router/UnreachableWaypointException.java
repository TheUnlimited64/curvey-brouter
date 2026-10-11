package btools.router;

/**
 * Raised when a waypoint cannot be reached because it sits on a disconnected road island.
 * Extends {@link IllegalArgumentException} and keeps the historic message ("target island detected
 * for section N") so callers that only read {@link RoutingEngine#getErrorMessage()} are unaffected;
 * typed callers read {@link RoutingEngine#getUnreachableWaypointIndex()} instead of parsing text.
 */
public class UnreachableWaypointException extends IllegalArgumentException {
  private final int waypointIndex;

  /**
   * @param waypointIndex index of the unreachable waypoint in the list the engine was given
   * @param message       human-readable description
   */
  public UnreachableWaypointException(int waypointIndex, String message) {
    super(message);
    this.waypointIndex = waypointIndex;
  }

  public int getWaypointIndex() {
    return waypointIndex;
  }
}
