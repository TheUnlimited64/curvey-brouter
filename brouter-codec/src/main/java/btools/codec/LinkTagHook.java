package btools.codec;

/**
 * EXPERIMENT (#341, Curvey fork): lets the decoder replace a link's way description while a tile cell is woven,
 * e.g. to add a value from a side-car file next to the routing tile. Keyed by the link's two end nodes.
 */
public interface LinkTagHook {
  /** Returns the description to store for the link from (ilonA, ilatA) to (ilonB, ilatB), or {@code description}. */
  byte[] tagLink(int ilonA, int ilatA, int ilonB, int ilatB, byte[] description);
}
