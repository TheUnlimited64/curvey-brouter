package btools.codec;

/**
 * Curvey: lets the engine change a link's way description while a routing cell is decoded, e.g. to add the
 * {@code curvey_curve_class} pseudo-tag from a curvature tile next to the routing tile.
 */
public interface LinkTagHook {
  /**
   * Called once per decoded link that the profile can use.
   *
   * @param ilonA       node being decoded (microdegrees, lon + 180)
   * @param ilatA       node being decoded (microdegrees, lat + 90)
   * @param ilonB       other end of the link
   * @param ilatB       other end of the link
   * @param description the link's way description as stored in the routing tile
   * @return the description to use: {@code description} itself when nothing changes, else a unified variant
   */
  byte[] tagLink(int ilonA, int ilatA, int ilonB, int ilatB, byte[] description);
}
