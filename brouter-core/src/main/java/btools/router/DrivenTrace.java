package btools.router;

import java.util.ArrayList;
import java.util.List;

import btools.mapaccess.OsmPos;
import btools.util.CheapRuler;

/**
 * The road path driven (or routed) before a leg's start, oldest first, in engine coordinates (#204): consecutive
 * positions are the steps the vehicle drove. Rule R2 of Curvey {@code docs/engine-driven-trace.md}: a route step that runs
 * along one of these steps in the opposite direction is going back the way the vehicle came, and costs extra
 * ({@link RoutingContext#reversalCostFactor}).
 *
 * <p>The test is geometric with a tolerance of {@link #ON_STEP_TOLERANCE_M}, not a lookup of node pairs: the
 * positions come from the engine's own geometry, but a leg start or a matched position lies inside a road
 * segment, and its rounding differs by a few centimetres from the crosspoint the next search computes. One metre
 * is far below the distance between two different roads, so no neighbouring road, no hairpin leg and no parallel
 * carriageway is ever taken for the trace.
 */
final class DrivenTrace {
  /** Distance from a driven step's line within which a route step counts as running along it: rounding only. */
  static final double ON_STEP_TOLERANCE_M = 1.;

  /** Overlap below this is a route step touching the trace at a point (crossing it at a junction), not following it. */
  static final double MIN_OVERLAP_M = 1.;

  /**
   * Margin (micro degrees, about 2 m) of the per-step box pre-check in {@link #backwardOverlapM}: wider than
   * {@link #ON_STEP_TOLERANCE_M} at any latitude BRouter covers, so the pre-check never rejects a real overlap.
   */
  private static final int STEP_BOX_MARGIN = 20;

  private final int[] lons;
  private final int[] lats;
  private final double lengthM;

  // bounding box of all positions, widened by the tolerance (micro degrees)
  private final int minLon;
  private final int maxLon;
  private final int minLat;
  private final int maxLat;

  private DrivenTrace(int[] lons, int[] lats) {
    this.lons = lons;
    this.lats = lats;
    double len = 0;
    int x0 = Integer.MAX_VALUE, x1 = Integer.MIN_VALUE, y0 = Integer.MAX_VALUE, y1 = Integer.MIN_VALUE;
    for (int i = 0; i < lons.length; i++) {
      if (i > 0) len += CheapRuler.distance(lons[i - 1], lats[i - 1], lons[i], lats[i]);
      x0 = Math.min(x0, lons[i]);
      x1 = Math.max(x1, lons[i]);
      y0 = Math.min(y0, lats[i]);
      y1 = Math.max(y1, lats[i]);
    }
    lengthM = len;
    double[] k = CheapRuler.getLonLatToMeterScales((y0 + y1) >> 1);
    int marginLon = (int) Math.ceil(ON_STEP_TOLERANCE_M / k[0]) + 1;
    int marginLat = (int) Math.ceil(ON_STEP_TOLERANCE_M / k[1]) + 1;
    minLon = x0 - marginLon;
    maxLon = x1 + marginLon;
    minLat = y0 - marginLat;
    maxLat = y1 + marginLat;
  }

  /**
   * The last {@code maxLengthM} of {@code positions} (oldest first), cut inside the step that crosses the limit;
   * consecutive duplicates are dropped. Null when fewer than two distinct positions remain.
   */
  static DrivenTrace tail(List<? extends OsmPos> positions, double maxLengthM) {
    if (positions == null || positions.size() < 2) return null;
    List<int[]> kept = new ArrayList<>();
    double len = 0;
    OsmPos newer = null;
    for (int i = positions.size() - 1; i >= 0; i--) {
      OsmPos p = positions.get(i);
      if (newer != null && p.getILon() == newer.getILon() && p.getILat() == newer.getILat()) continue;
      if (newer != null) {
        double step = CheapRuler.distance(p.getILon(), p.getILat(), newer.getILon(), newer.getILat());
        if (len + step > maxLengthM) {
          double f = (maxLengthM - len) / step;
          kept.add(new int[]{
            newer.getILon() + (int) Math.round(f * (p.getILon() - newer.getILon())),
            newer.getILat() + (int) Math.round(f * (p.getILat() - newer.getILat()))});
          break;
        }
        len += step;
      }
      kept.add(new int[]{p.getILon(), p.getILat()});
      newer = p;
    }
    if (kept.size() < 2) return null;
    int n = kept.size();
    int[] lons = new int[n];
    int[] lats = new int[n];
    for (int i = 0; i < n; i++) {
      int[] p = kept.get(n - 1 - i);
      lons[i] = p[0];
      lats[i] = p[1];
    }
    return new DrivenTrace(lons, lats);
  }

  int size() {
    return lons.length;
  }

  int lon(int i) {
    return lons[i];
  }

  int lat(int i) {
    return lats[i];
  }

  double lengthM() {
    return lengthM;
  }

  /**
   * Length (m) of the route step {@code from -> to} that runs along a driven step in the opposite direction:
   * both ends within {@link #ON_STEP_TOLERANCE_M} of the driven step's line, moving from its end toward its
   * start, overlapping it by at least {@link #MIN_OVERLAP_M}. Zero for any other step (forward along the trace,
   * crossing it, another road).
   */
  double backwardOverlapM(int fromLon, int fromLat, int toLon, int toLat) {
    if (Math.max(fromLon, toLon) < minLon || Math.min(fromLon, toLon) > maxLon
      || Math.max(fromLat, toLat) < minLat || Math.min(fromLat, toLat) > maxLat) {
      return 0.;
    }
    double[] k = CheapRuler.getLonLatToMeterScales((fromLat + toLat) >> 1);
    double total = 0.;
    for (int i = 1; i < lons.length; i++) {
      int aLon = lons[i - 1], aLat = lats[i - 1], bLon = lons[i], bLat = lats[i];
      // cheap reject: the route step's box does not touch the driven step's box
      if (Math.max(fromLon, toLon) < Math.min(aLon, bLon) - STEP_BOX_MARGIN
        || Math.min(fromLon, toLon) > Math.max(aLon, bLon) + STEP_BOX_MARGIN
        || Math.max(fromLat, toLat) < Math.min(aLat, bLat) - STEP_BOX_MARGIN
        || Math.min(fromLat, toLat) > Math.max(aLat, bLat) + STEP_BOX_MARGIN) {
        continue;
      }
      double ux = (bLon - aLon) * k[0], uy = (bLat - aLat) * k[1];
      double len2 = ux * ux + uy * uy;
      if (len2 == 0.) continue;
      double len = Math.sqrt(len2);
      double fx = (fromLon - aLon) * k[0], fy = (fromLat - aLat) * k[1];
      double tx = (toLon - aLon) * k[0], ty = (toLat - aLat) * k[1];
      if (Math.abs(fx * uy - fy * ux) / len > ON_STEP_TOLERANCE_M) continue;
      if (Math.abs(tx * uy - ty * ux) / len > ON_STEP_TOLERANCE_M) continue;
      double tFrom = (fx * ux + fy * uy) / len2;
      double tTo = (tx * ux + ty * uy) / len2;
      if (tTo >= tFrom) continue; // forward along the trace
      double overlap = (Math.min(tFrom, 1.) - Math.max(tTo, 0.)) * len;
      if (overlap >= MIN_OVERLAP_M) total += overlap;
    }
    double stepLen = CheapRuler.distance(fromLon, fromLat, toLon, toLat);
    return Math.min(total, stepLen);
  }
}
