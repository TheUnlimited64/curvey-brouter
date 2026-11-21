package btools.router;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import btools.mapaccess.OsmPos;
import btools.util.CheapRuler;

/**
 * A line the route should follow in its direction (Curvey #458), oldest position first, in engine coordinates:
 * Curvey sends an old route played the other way round when the rider reverses it. Request key
 * {@code referencetrack} ({@link RoutingContext#referenceTrackPositions}).
 *
 * <p>While a leg follows the line (both of its ends lie on it, the end after the start, {@link RoutingEngine}), the
 * part of a route step that does not run along it costs {@link RoutingContext#referenceDeviationCostFactor} times
 * its normal cost. Access, one-way rules and turn restrictions stay as hard as ever, so a route leaves the line only
 * where they make it (a one-way street, a roundabout, a turn restriction) and comes back as soon as it can. The line
 * is not made cheaper instead: a step would then cost less than the profile's minimum per metre, which BRouter's
 * search relies on (the {@code pass1coefficient} estimate, the {@code cost + airdistance} cut-off).
 *
 * <p>"Along" is geometric with a corridor of {@link #ALONG_CORRIDOR_M}, not a lookup of node pairs: the line may
 * come from a stored route (rounded to 1e-5 degrees) or from another engine with other map data, and the other
 * direction of a divided road runs on its own carriageway next to the line. Only the line's own direction counts:
 * a step against it, a road crossing it or a turn off it is off the line.
 */
final class ReferenceTrack {
  /**
   * Distance from the line within which a step in its direction follows it: wider than the offset of the two
   * carriageways of most divided roads, which the other direction has to take, narrower than the distance between
   * two parallel streets. The corridor Curvey's {@code RouteDivergence} counts as the same road. A boulevard with a
   * wide median lies farther apart (Unter den Linden in Berlin: about 31 m); there the other carriageway counts as
   * off the line, and the route still takes it where no parallel road is cheaper.
   */
  static final double ALONG_CORRIDOR_M = 30.;

  /**
   * Largest angle (degrees) between a step and the line for the step to run along it. A carriageway leaving its twin
   * or a road bending away stays well below; a turn into a side road or a road crossing the line is far above.
   */
  static final double MAX_ALONG_ANGLE_DEG = 30.;

  private static final double MIN_ALONG_COS = Math.cos(Math.toRadians(MAX_ALONG_ANGLE_DEG));

  /** Grid cell size, micro degrees (about 220 m north-south): a line of a few thousand steps has a few in each. */
  private static final int GRID_CELL = 2_000;

  private final int[] lons;
  private final int[] lats;
  /** Distance along the line (m) of each position. */
  private final double[] posM;

  /** {@link #ALONG_CORRIDOR_M} in micro degrees at the line's latitudes, rounded up. */
  private final int marginLon;
  private final int marginLat;

  // bounding box of all positions, widened by the corridor (micro degrees)
  private final int minLon;
  private final int maxLon;
  private final int minLat;
  private final int maxLat;

  /**
   * Grid of the line's steps (index of their end point), each listed in every cell its box widened by the corridor
   * touches, so a query needs only the cells its own box touches: sorted cell keys, and per cell its steps in
   * {@code cellSteps[cellFirst[c]..cellFirst[c + 1])}.
   */
  private final long[] cellKeys;
  private final int[] cellFirst;
  private final int[] cellSteps;

  /** Per step the query that saw it last, so a step in several cells of one query counts once. */
  private final int[] seenInQuery;
  private int query;

  private ReferenceTrack(int[] lons, int[] lats) {
    this.lons = lons;
    this.lats = lats;
    posM = new double[lons.length];
    int x0 = Integer.MAX_VALUE, x1 = Integer.MIN_VALUE, y0 = Integer.MAX_VALUE, y1 = Integer.MIN_VALUE;
    for (int i = 0; i < lons.length; i++) {
      if (i > 0) posM[i] = posM[i - 1] + CheapRuler.distance(lons[i - 1], lats[i - 1], lons[i], lats[i]);
      x0 = Math.min(x0, lons[i]);
      x1 = Math.max(x1, lons[i]);
      y0 = Math.min(y0, lats[i]);
      y1 = Math.max(y1, lats[i]);
    }
    // the poleward edge has the shortest degree of longitude, so the widest margin
    int polewardLat = Math.abs(y0 - 90_000_000) > Math.abs(y1 - 90_000_000) ? y0 : y1;
    double[] k = CheapRuler.getLonLatToMeterScales(polewardLat);
    marginLon = (int) Math.ceil(ALONG_CORRIDOR_M / k[0]) + 1;
    marginLat = (int) Math.ceil(ALONG_CORRIDOR_M / k[1]) + 1;
    minLon = x0 - marginLon;
    maxLon = x1 + marginLon;
    minLat = y0 - marginLat;
    maxLat = y1 + marginLat;
    seenInQuery = new int[lons.length];

    Map<Long, List<Integer>> cells = new HashMap<>();
    for (int i = 1; i < lons.length; i++) {
      // a long step in pieces of at most a cell, so it costs cells by its length, not by the area of its box
      int pieces = 1 + Math.max(Math.abs(lons[i] - lons[i - 1]), Math.abs(lats[i] - lats[i - 1])) / GRID_CELL;
      for (int p = 0; p < pieces; p++) {
        int aLon = lons[i - 1] + (int) ((long) (lons[i] - lons[i - 1]) * p / pieces);
        int aLat = lats[i - 1] + (int) ((long) (lats[i] - lats[i - 1]) * p / pieces);
        int bLon = lons[i - 1] + (int) ((long) (lons[i] - lons[i - 1]) * (p + 1) / pieces);
        int bLat = lats[i - 1] + (int) ((long) (lats[i] - lats[i - 1]) * (p + 1) / pieces);
        for (int cx = (Math.min(aLon, bLon) - marginLon) / GRID_CELL; cx <= (Math.max(aLon, bLon) + marginLon) / GRID_CELL; cx++) {
          for (int cy = (Math.min(aLat, bLat) - marginLat) / GRID_CELL; cy <= (Math.max(aLat, bLat) + marginLat) / GRID_CELL; cy++) {
            List<Integer> steps = cells.computeIfAbsent(cellKey(cx, cy), c -> new ArrayList<>());
            if (steps.isEmpty() || steps.get(steps.size() - 1) != i) steps.add(i);
          }
        }
      }
    }
    cellKeys = new long[cells.size()];
    int c = 0;
    for (Long key : cells.keySet()) cellKeys[c++] = key;
    Arrays.sort(cellKeys);
    cellFirst = new int[cellKeys.length + 1];
    int total = 0;
    for (c = 0; c < cellKeys.length; c++) {
      cellFirst[c] = total;
      total += cells.get(cellKeys[c]).size();
    }
    cellFirst[cellKeys.length] = total;
    cellSteps = new int[total];
    for (c = 0; c < cellKeys.length; c++) {
      List<Integer> steps = cells.get(cellKeys[c]);
      for (int j = 0; j < steps.size(); j++) cellSteps[cellFirst[c] + j] = steps.get(j);
    }
  }

  private static long cellKey(int cx, int cy) {
    return ((long) cx << 32) ^ (cy & 0xffffffffL);
  }

  /** The line through {@code positions} in their order, consecutive duplicates dropped; null with fewer than two. */
  static ReferenceTrack of(List<? extends OsmPos> positions) {
    if (positions == null) return null;
    int[] lons = new int[positions.size()];
    int[] lats = new int[positions.size()];
    int n = 0;
    for (OsmPos p : positions) {
      if (n > 0 && p.getILon() == lons[n - 1] && p.getILat() == lats[n - 1]) continue;
      lons[n] = p.getILon();
      lats[n] = p.getILat();
      n++;
    }
    return n < 2 ? null : new ReferenceTrack(Arrays.copyOf(lons, n), Arrays.copyOf(lats, n));
  }

  int size() {
    return lons.length;
  }

  double lengthM() {
    return posM[posM.length - 1];
  }

  /**
   * Length (m) of the route step {@code from -> to} that runs along the line in its direction: within
   * {@link #ALONG_CORRIDOR_M} of one of its steps, at most {@link #MAX_ALONG_ANGLE_DEG} off that step's direction.
   * At most the step's length; zero for a step against the line, across it, or away from it.
   */
  double alongM(int fromLon, int fromLat, int toLon, int toLat) {
    if (Math.max(fromLon, toLon) < minLon || Math.min(fromLon, toLon) > maxLon
      || Math.max(fromLat, toLat) < minLat || Math.min(fromLat, toLat) > maxLat) {
      return 0.;
    }
    double[] k = CheapRuler.getLonLatToMeterScales((fromLat + toLat) >> 1);
    double sx = (toLon - fromLon) * k[0];
    double sy = (toLat - fromLat) * k[1];
    double stepLen = Math.sqrt(sx * sx + sy * sy);
    if (stepLen == 0.) return 0.;
    int q = nextQuery();
    double total = 0.;
    // the cells under the step, in pieces of at most a cell: a long diagonal step costs cells by its length, not by
    // the area of its box (each line step near a point of the route step is listed in that point's cell)
    int pieces = 1 + Math.max(Math.abs(toLon - fromLon), Math.abs(toLat - fromLat)) / GRID_CELL;
    for (int p = 0; p < pieces; p++) {
      int aLon = fromLon + (int) ((long) (toLon - fromLon) * p / pieces);
      int aLat = fromLat + (int) ((long) (toLat - fromLat) * p / pieces);
      int bLon = fromLon + (int) ((long) (toLon - fromLon) * (p + 1) / pieces);
      int bLat = fromLat + (int) ((long) (toLat - fromLat) * (p + 1) / pieces);
      for (int cx = Math.min(aLon, bLon) / GRID_CELL; cx <= Math.max(aLon, bLon) / GRID_CELL; cx++) {
        for (int cy = Math.min(aLat, bLat) / GRID_CELL; cy <= Math.max(aLat, bLat) / GRID_CELL; cy++) {
          int c = Arrays.binarySearch(cellKeys, cellKey(cx, cy));
          if (c < 0) continue;
          for (int j = cellFirst[c]; j < cellFirst[c + 1]; j++) {
            int i = cellSteps[j];
            if (seenInQuery[i] == q) continue;
            seenInQuery[i] = q;
            total += stepAlong(i, fromLon, fromLat, toLon, toLat, sx, sy, stepLen, k);
          }
        }
      }
    }
    return Math.min(total, stepLen);
  }

  /**
   * Length (m) of the route step {@code from -> to} that runs along the line in either direction (Curvey #466, a
   * stretch to avoid): {@link #alongM} of the step plus that of the step the other way round. A road crossing the
   * line, or a turn off it, still counts as off.
   */
  double alongEitherWayM(int fromLon, int fromLat, int toLon, int toLat) {
    return alongM(fromLon, fromLat, toLon, toLat) + alongM(toLon, toLat, fromLon, fromLat);
  }

  /**
   * Length (m) of the route step that runs along line step {@code i} (from position {@code i - 1} to {@code i}): the
   * part of the route step inside the corridor whose projection falls on the line step.
   */
  private double stepAlong(int i, int fromLon, int fromLat, int toLon, int toLat, double sx, double sy, double stepLen,
                           double[] k) {
    int aLon = lons[i - 1], aLat = lats[i - 1], bLon = lons[i], bLat = lats[i];
    // cheap reject: the route step's box does not touch the line step's box widened by the corridor
    if (Math.max(fromLon, toLon) < Math.min(aLon, bLon) - marginLon
      || Math.min(fromLon, toLon) > Math.max(aLon, bLon) + marginLon
      || Math.max(fromLat, toLat) < Math.min(aLat, bLat) - marginLat
      || Math.min(fromLat, toLat) > Math.max(aLat, bLat) + marginLat) {
      return 0.;
    }
    double ux = (bLon - aLon) * k[0], uy = (bLat - aLat) * k[1];
    double len2 = ux * ux + uy * uy;
    if (len2 == 0.) return 0.;
    double len = Math.sqrt(len2);
    if (ux * sx + uy * sy < MIN_ALONG_COS * len * stepLen) return 0.; // against, across or off the line's direction
    double fx = (fromLon - aLon) * k[0], fy = (fromLat - aLat) * k[1];
    double tx = (toLon - aLon) * k[0], ty = (toLat - aLat) * k[1];
    // signed distance from the line step's line, and position along it (0 at its start, 1 at its end), both linear
    // in s, the share of the route step from its start
    double dFrom = (fx * uy - fy * ux) / len, dTo = (tx * uy - ty * ux) / len;
    double pFrom = (fx * ux + fy * uy) / len2, pTo = (tx * ux + ty * uy) / len2;
    double lo = 0., hi = 1.;
    if (dTo == dFrom) {
      if (Math.abs(dFrom) > ALONG_CORRIDOR_M) return 0.;
    } else {
      double s1 = (ALONG_CORRIDOR_M - dFrom) / (dTo - dFrom);
      double s2 = (-ALONG_CORRIDOR_M - dFrom) / (dTo - dFrom);
      lo = Math.max(lo, Math.min(s1, s2));
      hi = Math.min(hi, Math.max(s1, s2));
    }
    double dp = pTo - pFrom; // > 0: the direction test passed
    lo = Math.max(lo, -pFrom / dp);
    hi = Math.min(hi, (1. - pFrom) / dp);
    return hi > lo ? (hi - lo) * stepLen : 0.;
  }

  /**
   * Where the line passes within {@link #ALONG_CORRIDOR_M} of a position: {first, last} distance along the line (m)
   * of its nearest points, or null when it does not pass there.
   */
  double[] passesNear(int lon, int lat) {
    if (lon < minLon || lon > maxLon || lat < minLat || lat > maxLat) return null;
    int c = Arrays.binarySearch(cellKeys, cellKey(lon / GRID_CELL, lat / GRID_CELL));
    if (c < 0) return null;
    double[] k = CheapRuler.getLonLatToMeterScales(lat);
    double first = Double.POSITIVE_INFINITY, last = Double.NEGATIVE_INFINITY;
    for (int j = cellFirst[c]; j < cellFirst[c + 1]; j++) {
      int i = cellSteps[j];
      double ux = (lons[i] - lons[i - 1]) * k[0], uy = (lats[i] - lats[i - 1]) * k[1];
      double px = (lon - lons[i - 1]) * k[0], py = (lat - lats[i - 1]) * k[1];
      double len2 = ux * ux + uy * uy;
      double t = len2 == 0. ? 0. : Math.max(0., Math.min(1., (px * ux + py * uy) / len2));
      double ex = px - t * ux, ey = py - t * uy;
      if (Math.sqrt(ex * ex + ey * ey) > ALONG_CORRIDOR_M) continue;
      double pos = posM[i - 1] + t * (posM[i] - posM[i - 1]);
      first = Math.min(first, pos);
      last = Math.max(last, pos);
    }
    return first > last ? null : new double[]{first, last};
  }

  private int nextQuery() {
    if (++query == Integer.MAX_VALUE) { // after two billion queries: start the marks again
      Arrays.fill(seenInQuery, 0);
      query = 1;
    }
    return query;
  }
}
