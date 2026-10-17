package btools.router;

/**
 * One position a vehicle was at before a leg's start (#204), with its 1-sigma error: a GPS fix with its reported
 * accuracy, or a point on the route the vehicle was following, which is known much better. Input of
 * {@link DrivenTraceMatcher}. Request key {@code driventrace=lon,lat,sigma|...}, oldest first.
 */
public final class TraceObservation {
  /** Smallest error accepted: below it the road geometry itself (rounding, carriageway width) dominates. */
  static final double MIN_SIGMA_M = 1.;

  /** BRouter micro-degree offsets, like {@link btools.mapaccess.OsmNode}. */
  public final int ilon;
  public final int ilat;
  public final double sigmaM;

  public TraceObservation(int ilon, int ilat, double sigmaM) {
    this.ilon = ilon;
    this.ilat = ilat;
    this.sigmaM = Math.max(MIN_SIGMA_M, sigmaM);
  }
}
