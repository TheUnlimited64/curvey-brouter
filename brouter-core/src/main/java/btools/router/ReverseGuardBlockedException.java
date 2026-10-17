package btools.router;

/**
 * Raised when a leg finds no track while the no-U-turn guard ({@code continueStraight}) rejected at
 * least one of its first steps. Extends {@link IllegalArgumentException} and keeps the historic message
 * ("no track found at pass=N (without a U-turn at ...)") so callers that read
 * {@link RoutingEngine#getErrorMessage()} are unaffected.
 */
public class ReverseGuardBlockedException extends IllegalArgumentException {
  public ReverseGuardBlockedException(String message) {
    super(message);
  }
}
