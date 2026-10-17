package btools.router;

import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.PriorityQueue;

import btools.codec.WaypointMatcher;
import btools.mapaccess.MatchedWaypoint;
import btools.mapaccess.NodesCache;
import btools.mapaccess.OsmLink;
import btools.mapaccess.OsmNode;
import btools.mapaccess.OsmNodePairSet;
import btools.mapaccess.OsmTransferNode;
import btools.util.CheapRuler;

/**
 * Reconstructs the road path a vehicle drove from the positions it reported (#204, Curvey {@code docs/engine-driven-trace.md}):
 * a hidden Markov model over the road graph (Newson and Krumm 2009), solved with Viterbi.
 *
 * <ul>
 * <li>States: per observation, the closest routable links within {@link #radiusM}, each in every direction the
 * profile allows: a position on a link plus a travel direction.</li>
 * <li>Emission: Gaussian in the distance between the observation and the link, scaled by the observation's sigma.</li>
 * <li>Transition: the road distance between two states (driving in the first state's direction, turning only at a
 * dead end) against the straight distance between the two observations. A turnaround on the link costs
 * {@link #TURNAROUND_LOG_PENALTY}; a small step backwards on the same link is GPS noise.</li>
 * </ul>
 *
 * <p>The result is the most likely road path, cut at the last turnaround (before it the vehicle drove the way it is
 * going now), and the vehicle's position and direction at the last observation. No heading and no angle threshold
 * enter: a sharp corner is road geometry like any other.
 */
final class DrivenTraceMatcher {
  /** Only the newest this many observations are used: the arrival is what matters, and the cost stays bounded. */
  static final int MAX_OBSERVATIONS = 60;

  /** Links up to this many sigmas from an observation are candidates (99.7% of a Gaussian error). */
  static final double RADIUS_SIGMAS = 3.;

  /** Smallest candidate radius: road width and the offset of a mapped centre line, whatever the sigma. */
  static final double MIN_RADIUS_M = 10.;

  /** Largest candidate radius: beyond it a fix says nothing about the road, and the search would grow. */
  static final double MAX_RADIUS_M = 60.;

  /** Closest links kept per observation; more only add states that never win. */
  static final int MAX_LINKS_PER_OBSERVATION = 5;

  /**
   * Scale of the transition term {@code -|D - g| / (TRANSITION_BETA_M + TRANSITION_BETA_PER_M * g)}: road distance
   * {@code D} against straight distance {@code g}. The fixed part is GPS error on short steps; the relative part a
   * winding road between two fixes (a 30 m chord through a hairpin can be 45 m of road: cost 2.5). It stays small:
   * across a long gap a detour of most of the gap must cost more than dropping a fix ({@link #OUTLIER_LOG_PENALTY}).
   */
  static final double TRANSITION_BETA_M = 3.;
  static final double TRANSITION_BETA_PER_M = 0.1;

  /**
   * A road path longer than {@code MAX_DETOUR_FACTOR * g + MAX_DETOUR_SLACK_M} plus both candidate radii is not
   * considered at all: its transition probability is negligible, and the bound keeps each search local.
   */
  static final double MAX_DETOUR_FACTOR = 2.;
  static final double MAX_DETOUR_SLACK_M = 30.;

  /**
   * Log-probability cost of turning around on a link (about 1 in 400). A vehicle that did turn produces fixes that
   * only fit the turn, so the turn still wins; GPS jitter alone does not pay for it.
   */
  static final double TURNAROUND_LOG_PENALTY = 6.;

  /**
   * Log-probability cost of an observation the path does not explain: an outlier in the middle (a GPS jump onto a
   * road the vehicle never drove), or one before the path's start (the path may begin at any observation). About
   * e^-5, as unlikely as a fix 3.2 sigma off. One cost for both, because both say the same thing about the fix: a
   * cheaper late start once let the match drop every fix of a receiver offset 2 sigma sideways (multipath, a bias
   * it does not report) and keep only the start, so the direction was lost.
   */
  static final double OUTLIER_LOG_PENALTY = 5.;

  /**
   * Outliers in a row that a transition may jump over. More than that is not a GPS jump but another road: the path
   * then starts after them ({@link #OUTLIER_LOG_PENALTY} each).
   */
  static final int MAX_SKIPPED_OBSERVATIONS = 2;

  /**
   * The vehicle's direction at the last observation is known only when it is this much more likely (log-odds,
   * about 20:1) than the opposite direction on the same link. One observation alone never gets there.
   */
  static final double DIRECTION_MIN_LOG_ODDS = 3.;

  /**
   * The start leaves the engine's own match (the nearest road) only when the path ends on another link this much
   * more likely (log-odds, about 7:1): next to a junction a noisy fix fits both, and moving then gains nothing.
   * Lower than {@link #DIRECTION_MIN_LOG_ODDS}: a start left on a wrong nearest road is what the engine did
   * before, a guard armed the wrong way forbids the way ahead.
   */
  static final double START_MOVE_MIN_LOG_ODDS = 2.;

  /** Default sigma of the start when the observations do not end there (a phone GPS fix). */
  static final double DEFAULT_START_SIGMA_M = 10.;

  /** The last observation counts as the start itself within this distance (rounding of the request). */
  static final double SAME_POSITION_M = 1.;

  /** Links are collected from map cells within this distance of an observation: a link's source node can lie that far away. */
  static final double CELL_REACH_M = 500.;

  /** Costfactor from which the profile forbids a way in that direction (same threshold as {@link OsmPath}). */
  private static final float FORBIDDEN_COSTFACTOR = 9998.f;

  /** Result of a match. */
  static final class Match {
    /** Road path up to the start, oldest first, cut at the last turnaround and to its last 500 m; null when no step is known. */
    final DrivenTrace trace;
    /**
     * Where the start moves (crosspoint, node1 behind the vehicle, node2 ahead), or null: the engine's own match is
     * as likely ({@link #START_MOVE_MIN_LOG_ODDS}) and stays.
     */
    final MatchedWaypoint start;
    /** True when the travel direction at the end of the path is clear ({@link #DIRECTION_MIN_LOG_ODDS}). */
    final boolean directionKnown;
    final double directionLogOdds;
    /** The most likely path, oldest first, cut at the last turnaround, not cut in length. */
    final List<OsmNode> path;
    /** The nearest vertex behind the path's end on its last link, more than the via tolerance away; null if none. */
    final OsmNode behindOnLink;
    /** Observations on the path (after outliers and a late start), for diagnostics. */
    final int observationsUsed;

    Match(DrivenTrace trace, MatchedWaypoint start, boolean directionKnown, double directionLogOdds, List<OsmNode> path,
          OsmNode behindOnLink, int observationsUsed) {
      this.trace = trace;
      this.start = start;
      this.directionKnown = directionKnown;
      this.directionLogOdds = directionLogOdds;
      this.path = path;
      this.behindOnLink = behindOnLink;
      this.observationsUsed = observationsUsed;
    }

    /**
     * The position R1 guards for a leg starting at {@code x}: the newest path position more than the via tolerance
     * from it, else the vertex behind on the last link. Null when there is none. When the start stayed on another
     * road than the path's end, this position does not lie on any first step from the start, so the guard never
     * fires: an unclear position guards nothing rather than the wrong way.
     */
    OsmNode behind(OsmNode x) {
      double tol = RoutingContext.REVERSE_GUARD_VIA_TOLERANCE_M;
      for (int i = path.size() - 1; i >= 0; i--) {
        OsmNode p = path.get(i);
        if (CheapRuler.distance(p.ilon, p.ilat, x.ilon, x.ilat) > tol) return p;
      }
      if (behindOnLink != null && CheapRuler.distance(behindOnLink.ilon, behindOnLink.ilat, x.ilon, x.ilat) > tol) return behindOnLink;
      return null;
    }
  }

  /** A link near an observation, with the observation projected onto its geometry. */
  private static final class Candidate {
    OsmLink link;
    OsmNode a; // geometry runs from a to b
    OsmNode b;
    int[] lons;
    int[] lats;
    double[] at; // distance from a at each vertex
    int cLon; // projection of the observation
    int cLat;
    double offset; // distance from a to the projection
    double dist; // observation to projection

    double length() {
      return at[at.length - 1];
    }
  }

  /** A candidate with a travel direction. */
  private static final class State {
    final Candidate c;
    final boolean forward; // travelling a -> b
    /** The same position in the other direction, if the profile allows it; the state after a turnaround here. */
    State twin;
    double score = Double.NEGATIVE_INFINITY;
    State prev;
    int kind; // how this state was reached from prev (or from prev's twin, if turned)
    /** The vehicle turned around at prev's position, then drove on from prev's twin. */
    boolean turned;
    Label arrival; // path from prev (or its twin), for KIND_ROAD

    State(Candidate c, boolean forward) {
      this.c = c;
      this.forward = forward;
    }

    OsmNode from() {
      return forward ? c.a : c.b;
    }

    OsmNode to() {
      return forward ? c.b : c.a;
    }

    /** Distance from {@link #from()} to the projection. */
    double pos() {
      return forward ? c.offset : c.length() - c.offset;
    }
  }

  private static final int KIND_START = 0; // first state of a chain (or a restart)
  private static final int KIND_AHEAD = 1; // same link, same direction, on or slightly behind (noise)
  private static final int KIND_ROAD = 2; // along other links

  /** Search label: arrived at {@code node} over {@code via} after {@code d} metres. */
  private static final class Label implements Comparable<Label> {
    final double d;
    final OsmNode node;
    final OsmLink via;
    final Label prev;

    Label(double d, OsmNode node, OsmLink via, Label prev) {
      this.d = d;
      this.node = node;
      this.via = via;
      this.prev = prev;
    }

    @Override
    public int compareTo(Label o) {
      return Double.compare(d, o.d);
    }
  }

  private final RoutingContext rc;
  private final java.util.function.Supplier<NodesCache> caches;
  /**
   * Links the engine has found to be islands (no route from or to them): no candidates, like for the engine's own
   * waypoint matching. Otherwise the start would be moved back onto one on every retry and the leg never routes.
   */
  private final OsmNodePairSet islandPairs;
  /** The graph the model runs on (the second of the two caches, see {@link #candidates}). */
  private NodesCache nodesCache;
  private final List<String> log;
  private final IdentityHashMap<OsmLink, Double> linkLengths = new IdentityHashMap<>();
  private final IdentityHashMap<OsmLink, Boolean> accessForward = new IdentityHashMap<>();
  private final IdentityHashMap<OsmLink, Boolean> accessReverse = new IdentityHashMap<>();

  /**
   * @param caches supplies fresh node caches (nothing decoded yet); the matcher closes them
   * @param log    receives one-line diagnostics
   */
  DrivenTraceMatcher(RoutingContext rc, java.util.function.Supplier<NodesCache> caches, OsmNodePairSet islandPairs,
                     List<String> log) {
    this.rc = rc;
    this.caches = caches;
    this.islandPairs = islandPairs;
    this.log = log;
  }

  static double radiusM(double sigmaM) {
    return Math.max(MIN_RADIUS_M, Math.min(MAX_RADIUS_M, RADIUS_SIGMAS * sigmaM));
  }

  /**
   * Matches {@code observations} (oldest first) ending at {@code start}, the engine's own match of the first
   * waypoint (its link is kept unless another one is clearly more likely). Null when the start cannot be placed on
   * a road (no candidate within its radius).
   */
  Match match(List<TraceObservation> observations, MatchedWaypoint start) {
    try {
      return matchOnGraph(observations, start);
    } finally {
      if (nodesCache != null) nodesCache.close();
      nodesCache = null;
    }
  }

  private Match matchOnGraph(List<TraceObservation> observations, MatchedWaypoint engineStart) {
    OsmNode startPos = engineStart.waypoint;
    List<TraceObservation> obs = new ArrayList<>(observations.subList(Math.max(0, observations.size() - MAX_OBSERVATIONS), observations.size()));
    TraceObservation last = obs.isEmpty() ? null : obs.get(obs.size() - 1);
    if (last == null || CheapRuler.distance(last.ilon, last.ilat, startPos.ilon, startPos.ilat) > SAME_POSITION_M) {
      obs.add(new TraceObservation(startPos.ilon, startPos.ilat, DEFAULT_START_SIGMA_M));
    } else {
      obs.set(obs.size() - 1, new TraceObservation(startPos.ilon, startPos.ilat, last.sigmaM));
    }
    int n = obs.size();
    // A start farther from the engine's snapped start than it looks for roads (a tapped point, a fix in a car park,
    // an over-confident accuracy) would have no road at all. The route starts where the engine snapped it, so that
    // is where the vehicle is; a start near roads keeps its own position, and the trace may move it onto another.
    TraceObservation startObs = obs.get(n - 1);
    OsmNode snapped = engineStart.crosspoint;
    if (snapped != null
      && CheapRuler.distance(startObs.ilon, startObs.ilat, snapped.ilon, snapped.ilat) > radiusM(startObs.sigmaM)) {
      obs.set(n - 1, new TraceObservation(snapped.ilon, snapped.ilat, startObs.sigmaM));
    }

    List<List<State>> states = candidates(obs);
    if (states.get(n - 1).isEmpty()) {
      log.add("driven trace: no road near the start");
      return null;
    }

    // Viterbi, forward, over the observations that have states (the others are off every road: skipped for free)
    List<Integer> withStates = new ArrayList<>();
    for (int j = 0; j < n; j++) {
      if (!states.get(j).isEmpty()) withStates.add(j);
    }
    for (int idx = 0; idx < withStates.size(); idx++) {
      int j = withStates.get(idx);
      List<State> next = states.get(j);
      double sigma = obs.get(j).sigmaM;
      // the path may start here, dropping every older observation
      for (State t : next) {
        offerStart(t, emission(t, sigma) - idx * OUTLIER_LOG_PENALTY);
      }
      // or continue from one of the previous observations, the ones in between being outliers
      for (int back = 1; back <= 1 + MAX_SKIPPED_OBSERVATIONS && idx - back >= 0; back++) {
        int i = withStates.get(idx - back);
        double g = CheapRuler.distance(obs.get(i).ilon, obs.get(i).ilat, obs.get(j).ilon, obs.get(j).ilat);
        double maxDist = MAX_DETOUR_FACTOR * g + MAX_DETOUR_SLACK_M + radiusM(obs.get(i).sigmaM) + radiusM(sigma);
        double beta = TRANSITION_BETA_M + TRANSITION_BETA_PER_M * g;
        double jitter = obs.get(i).sigmaM + sigma;
        double skipped = (back - 1) * OUTLIER_LOG_PENALTY;
        for (State s : states.get(i)) {
          if (s.score == Double.NEGATIVE_INFINITY) continue;
          transitions(s, skipped, next, maxDist, jitter, g, beta, sigma);
        }
      }
    }

    // the most likely end; its path gives the trace and the direction, whatever the start position below
    State best = null;
    State bestOnEngineLink = null;
    for (State t : states.get(n - 1)) {
      if (t.score == Double.NEGATIVE_INFINITY) continue;
      if (best == null || t.score > best.score) best = t;
      if (onLink(t, engineStart) && (bestOnEngineLink == null || t.score > bestOnEngineLink.score)) bestOnEngineLink = t;
    }
    if (best == null) {
      log.add("driven trace: start not reached");
      return null;
    }
    // the start moves off the engine's link only when the path is clearly elsewhere
    boolean moveStart = bestOnEngineLink == null || best.score - bestOnEngineLink.score >= START_MOVE_MIN_LOG_ODDS;
    // The reconstruction is the most likely path that ends where the route starts: on the engine's link when the
    // start stays there. The overall best may end on a neighbouring link (a parallel carriageway a few metres away);
    // its direction and the position behind it would then describe another road, and the guard could never fire.
    State end = moveStart ? best : bestOnEngineLink;
    double opposite = Double.NEGATIVE_INFINITY;
    boolean hasOpposite = false;
    for (State t : states.get(n - 1)) {
      if (t.c == end.c && t.forward != end.forward) {
        hasOpposite = true;
        opposite = t.score;
      }
    }
    double logOdds = hasOpposite ? end.score - opposite : Double.POSITIVE_INFINITY;
    boolean directionKnown = logOdds >= DIRECTION_MIN_LOG_ODDS;

    int used = 0;
    for (State s = end; s != null; s = s.prev) used++;
    List<OsmNode> positions = geometry(end);
    DrivenTrace trace = DrivenTrace.tail(positions, RoutingContext.DRIVEN_TRACE_MAX_LENGTH_M);

    MatchedWaypoint start = null;
    if (moveStart) {
      start = new MatchedWaypoint();
      start.crosspoint = new OsmNode(end.c.cLon, end.c.cLat);
      start.node1 = new OsmNode(end.from().ilon, end.from().ilat);
      start.node2 = new OsmNode(end.to().ilon, end.to().ilat);
      start.radius = end.c.dist;
    }

    log.add("driven trace: " + used + " of " + n + " observations, " + (trace == null ? 0 : (int) trace.lengthM()) + " m, direction log-odds "
      + (Double.isInfinite(logOdds) ? "inf" : String.format(java.util.Locale.ROOT, "%.1f", logOdds)) + ", path ends " + (int) end.c.dist + " m off the start");
    return new Match(trace, start, directionKnown, logOdds, positions, behindOnLink(end), used);
  }

  // ---------------------------------------------------------------------------------------------------------------
  // candidates

  /**
   * States per observation (index aligned with {@code obs}); the start (last) only on ways valid for a start.
   *
   * <p>Two caches: the first decodes the map cells around the observations with a link collector installed (the
   * decoder hands it every routable link once). The graph itself comes from a second one: BRouter's direct weaving
   * fills only nodes requested before their cell is decoded, so every candidate's end nodes are requested first.
   */
  private List<List<State>> candidates(List<TraceObservation> obs) {
    LinkCollector collector = new LinkCollector(obs, islandPairs);
    NodesCache collecting = caches.get();
    try {
      collecting.waypointMatcher = collector;
      for (TraceObservation o : obs) {
        double[] k = CheapRuler.getLonLatToMeterScales(o.ilat);
        int dLon = (int) (CELL_REACH_M / k[0]);
        int dLat = (int) (CELL_REACH_M / k[1]);
        for (int y = -1; y <= 1; y++) {
          for (int x = -1; x <= 1; x++) {
            collecting.loadSegmentFor(o.ilon + x * dLon, o.ilat + y * dLat);
          }
        }
      }
    } finally {
      collecting.waypointMatcher = null;
      collecting.close();
    }

    nodesCache = caches.get();
    // weaving fills these objects and then drops them from the cache's map: keep them here
    java.util.HashMap<Long, OsmNode> requested = new java.util.HashMap<>();
    for (List<RawLink> links : collector.found) {
      for (RawLink raw : links) {
        requested.computeIfAbsent(key(raw.lon1, raw.lat1), x -> nodesCache.getGraphNode(new OsmNode(raw.lon1, raw.lat1)));
        requested.computeIfAbsent(key(raw.lon2, raw.lat2), x -> nodesCache.getGraphNode(new OsmNode(raw.lon2, raw.lat2)));
      }
    }

    List<List<State>> result = new ArrayList<>();
    for (int i = 0; i < obs.size(); i++) {
      boolean isStart = i == obs.size() - 1;
      List<State> list = new ArrayList<>();
      for (RawLink raw : collector.found.get(i)) {
        if (isStart && !raw.startWay) continue;
        Candidate c = resolve(raw, requested.get(key(raw.lon1, raw.lat1)), obs.get(i));
        if (c == null) continue;
        State fwd = accessible(c.link, c.a) ? new State(c, true) : null;
        State bwd = accessible(c.link, c.b) ? new State(c, false) : null;
        if (fwd != null && bwd != null) {
          fwd.twin = bwd;
          bwd.twin = fwd;
        }
        if (fwd != null) list.add(fwd);
        if (bwd != null) list.add(bwd);
      }
      result.add(list);
    }
    return result;
  }

  /** {@code t} lies on the link the engine matched {@code wp} to (either direction). */
  private static boolean onLink(State t, MatchedWaypoint wp) {
    if (wp.node1 == null || wp.node2 == null) return false;
    OsmNode a = t.c.a;
    OsmNode b = t.c.b;
    return a.ilon == wp.node1.ilon && a.ilat == wp.node1.ilat && b.ilon == wp.node2.ilon && b.ilat == wp.node2.ilat
      || a.ilon == wp.node2.ilon && a.ilat == wp.node2.ilat && b.ilon == wp.node1.ilon && b.ilat == wp.node1.ilat;
  }

  private static long key(int lon, int lat) {
    return ((long) lon) << 32 | lat;
  }

  /** A link the decoder fed to the collector, near one observation. */
  private static final class RawLink {
    int lon1, lat1, lon2, lat2;
    int[] lons;
    int[] lats;
    double dist;
    boolean startWay;
  }

  /** Receives every routable link of the decoded map cells and keeps the closest ones per observation. */
  private static final class LinkCollector implements WaypointMatcher {
    final List<TraceObservation> obs;
    final OsmNodePairSet islandPairs;
    final double[] radius;
    final List<List<RawLink>> found = new ArrayList<>();
    private int minLon = Integer.MAX_VALUE, maxLon = Integer.MIN_VALUE, minLat = Integer.MAX_VALUE, maxLat = Integer.MIN_VALUE;
    private int[] lons = new int[16];
    private int[] lats = new int[16];
    private int count;
    private int lon1, lat1, lon2, lat2;
    private boolean startWay;

    LinkCollector(List<TraceObservation> obs, OsmNodePairSet islandPairs) {
      this.obs = obs;
      this.islandPairs = islandPairs;
      radius = new double[obs.size()];
      for (int i = 0; i < obs.size(); i++) {
        TraceObservation o = obs.get(i);
        radius[i] = radiusM(o.sigmaM);
        found.add(new ArrayList<>());
        double[] k = CheapRuler.getLonLatToMeterScales(o.ilat);
        int dLon = (int) (radius[i] / k[0]) + 1;
        int dLat = (int) (radius[i] / k[1]) + 1;
        minLon = Math.min(minLon, o.ilon - dLon);
        maxLon = Math.max(maxLon, o.ilon + dLon);
        minLat = Math.min(minLat, o.ilat - dLat);
        maxLat = Math.max(maxLat, o.ilat + dLat);
      }
    }

    @Override
    public boolean start(int ilonStart, int ilatStart, int ilonTarget, int ilatTarget, boolean useAsStartWay) {
      if (islandPairs != null && islandPairs.size() > 0
        && islandPairs.hasPair(((long) ilonStart) << 32 | ilatStart, ((long) ilonTarget) << 32 | ilatTarget)) {
        return false;
      }
      lon1 = ilonStart;
      lat1 = ilatStart;
      lon2 = ilonTarget;
      lat2 = ilatTarget;
      startWay = useAsStartWay;
      count = 0;
      add(ilonStart, ilatStart);
      return true;
    }

    @Override
    public void transferNode(int ilon, int ilat) {
      add(ilon, ilat);
    }

    @Override
    public void end() {
      add(lon2, lat2);
      int x0 = Integer.MAX_VALUE, x1 = Integer.MIN_VALUE, y0 = Integer.MAX_VALUE, y1 = Integer.MIN_VALUE;
      for (int i = 0; i < count; i++) {
        x0 = Math.min(x0, lons[i]);
        x1 = Math.max(x1, lons[i]);
        y0 = Math.min(y0, lats[i]);
        y1 = Math.max(y1, lats[i]);
      }
      if (x1 < minLon || x0 > maxLon || y1 < minLat || y0 > maxLat) return;
      for (int o = 0; o < obs.size(); o++) {
        TraceObservation p = obs.get(o);
        double d = distanceToPolyline(p.ilon, p.ilat, lons, lats, count);
        if (d > radius[o]) continue;
        List<RawLink> list = found.get(o);
        if (list.size() == MAX_LINKS_PER_OBSERVATION && list.get(list.size() - 1).dist <= d) continue;
        RawLink r = new RawLink();
        r.lon1 = lon1;
        r.lat1 = lat1;
        r.lon2 = lon2;
        r.lat2 = lat2;
        r.lons = java.util.Arrays.copyOf(lons, count);
        r.lats = java.util.Arrays.copyOf(lats, count);
        r.dist = d;
        r.startWay = startWay;
        int at = list.size();
        while (at > 0 && list.get(at - 1).dist > d) at--;
        list.add(at, r);
        if (list.size() > MAX_LINKS_PER_OBSERVATION) list.remove(list.size() - 1);
      }
    }

    @Override
    public boolean hasMatch(int lon, int lat) {
      return false;
    }

    private void add(int lon, int lat) {
      if (count == lons.length) {
        lons = java.util.Arrays.copyOf(lons, count * 2);
        lats = java.util.Arrays.copyOf(lats, count * 2);
      }
      lons[count] = lon;
      lats[count] = lat;
      count++;
    }
  }

  private static double distanceToPolyline(int lon, int lat, int[] lons, int[] lats, int count) {
    double[] k = CheapRuler.getLonLatToMeterScales(lat);
    double best = Double.MAX_VALUE;
    for (int i = 1; i < count; i++) {
      double ax = (lons[i - 1] - lon) * k[0], ay = (lats[i - 1] - lat) * k[1];
      double bx = (lons[i] - lon) * k[0], by = (lats[i] - lat) * k[1];
      double dx = bx - ax, dy = by - ay;
      double len2 = dx * dx + dy * dy;
      double t = len2 == 0 ? 0 : Math.max(0, Math.min(1, -(ax * dx + ay * dy) / len2));
      double x = ax + t * dx, y = ay + t * dy;
      best = Math.min(best, Math.sqrt(x * x + y * y));
    }
    return best;
  }

  /** The graph link behind {@code raw}, with the observation projected onto its geometry; null if not in the graph. */
  private Candidate resolve(RawLink raw, OsmNode a, TraceObservation o) {
    if (!nodesCache.obtainNonHollowNode(a)) return null;
    // the link between these two nodes whose geometry is the one the decoder fed (two ways may join the same nodes)
    OsmLink found = null;
    int[][] foundGeo = null;
    for (OsmLink l = a.firstlink; l != null; l = l.getNext(a)) {
      if (!isRoad(l, a)) continue;
      OsmNode b = l.getTarget(a);
      if (b.ilon != raw.lon2 || b.ilat != raw.lat2) continue;
      int[][] geo = geometry(l, a, b);
      boolean same = java.util.Arrays.equals(geo[0], raw.lons) && java.util.Arrays.equals(geo[1], raw.lats);
      if (found == null || same) {
        found = l;
        foundGeo = geo;
      }
      if (same) break;
    }
    if (found != null) {
      OsmLink l = found;
      OsmNode b = l.getTarget(a);
      int[][] geo = foundGeo;
      Candidate c = new Candidate();
      c.link = l;
      c.a = a;
      c.b = b;
      c.lons = geo[0];
      c.lats = geo[1];
      c.at = new double[c.lons.length];
      for (int i = 1; i < c.lons.length; i++) {
        c.at[i] = c.at[i - 1] + CheapRuler.distance(c.lons[i - 1], c.lats[i - 1], c.lons[i], c.lats[i]);
      }
      project(c, o.ilon, o.ilat);
      return c;
    }
    return null;
  }

  private static void project(Candidate c, int lon, int lat) {
    double[] k = CheapRuler.getLonLatToMeterScales(lat);
    double best = Double.MAX_VALUE;
    for (int i = 1; i < c.lons.length; i++) {
      double ax = (c.lons[i - 1] - lon) * k[0], ay = (c.lats[i - 1] - lat) * k[1];
      double bx = (c.lons[i] - lon) * k[0], by = (c.lats[i] - lat) * k[1];
      double dx = bx - ax, dy = by - ay;
      double len2 = dx * dx + dy * dy;
      double t = len2 == 0 ? 0 : Math.max(0, Math.min(1, -(ax * dx + ay * dy) / len2));
      double x = ax + t * dx, y = ay + t * dy;
      double d = Math.sqrt(x * x + y * y);
      if (d < best) {
        best = d;
        c.dist = d;
        c.cLon = c.lons[i - 1] + (int) Math.round(t * (c.lons[i] - c.lons[i - 1]));
        c.cLat = c.lats[i - 1] + (int) Math.round(t * (c.lats[i] - c.lats[i - 1]));
        c.offset = c.at[i - 1] + t * (c.at[i] - c.at[i - 1]);
      }
    }
  }

  // ---------------------------------------------------------------------------------------------------------------
  // model

  private static double emission(State s, double sigmaM) {
    double z = s.c.dist / sigmaM;
    return -0.5 * z * z;
  }

  /**
   * Relaxes every state of {@code next} from {@code s}: driving on, or turning around at {@code s}'s position
   * (at {@link #TURNAROUND_LOG_PENALTY}) and driving on from there; along the same link or the road network.
   */
  private void transitions(State s, double penalty, List<State> next, double maxDist, double jitter, double g, double beta, double sigmaNext) {
    relax(s, s, false, penalty, next, maxDist, jitter, g, beta, sigmaNext);
    if (s.twin != null) relax(s, s.twin, true, penalty, next, maxDist, jitter, g, beta, sigmaNext);
  }

  private void relax(State origin, State from, boolean turned, double penalty, List<State> next, double maxDist, double jitter, double g, double beta, double sigmaNext) {
    double base = origin.score - penalty - (turned ? TURNAROUND_LOG_PENALTY : 0.);
    double[] road = new double[next.size()];
    Label[] arrival = new Label[next.size()];
    java.util.Arrays.fill(road, Double.POSITIVE_INFINITY);
    search(from, next, maxDist, road, arrival);

    for (int k = 0; k < next.size(); k++) {
      State t = next.get(k);
      double em = emission(t, sigmaNext);
      if (t.c.link == from.c.link && t.forward == from.forward) {
        double ahead = t.pos() - from.pos();
        if (ahead >= -jitter && ahead <= maxDist) {
          // on along the link, or a step back that GPS noise explains (no movement)
          offer(t, origin, KIND_AHEAD, turned, null, base + em - Math.abs(Math.abs(ahead) - g) / beta);
        }
      }
      if (road[k] < Double.POSITIVE_INFINITY) {
        offer(t, origin, KIND_ROAD, turned, arrival[k], base + em - Math.abs(road[k] - g) / beta);
      }
    }
  }

  private static void offerStart(State t, double score) {
    if (score > t.score) {
      t.score = score;
      t.prev = null;
      t.kind = KIND_START;
      t.turned = false;
      t.arrival = null;
    }
  }

  private static void offer(State t, State from, int kind, boolean turned, Label arrival, double score) {
    if (score > t.score) {
      t.score = score;
      t.prev = from;
      t.kind = kind;
      t.turned = turned;
      t.arrival = arrival;
    }
  }

  /**
   * Shortest road distance from {@code s} (leaving its link at its end) to each state of {@code targets}, entering
   * the target's link at its start: a search over directed links that never turns back onto the link it came by,
   * except at a dead end. {@code road[k]} and {@code arrival[k]} receive the distance and the label at the target
   * link's start.
   */
  private void search(State s, List<State> targets, double maxDist, double[] road, Label[] arrival) {
    IdentityHashMap<OsmLink, List<Integer>> byLink = new IdentityHashMap<>();
    for (int k = 0; k < targets.size(); k++) {
      byLink.computeIfAbsent(targets.get(k).c.link, x -> new ArrayList<>()).add(k);
    }
    PriorityQueue<Label> open = new PriorityQueue<>();
    // directed links already settled: bit 1 = arrived at the link's forward end, bit 2 = at its reverse end
    IdentityHashMap<OsmLink, Integer> settled = new IdentityHashMap<>();
    open.add(new Label(s.c.length() - s.pos(), s.to(), s.c.link, null));
    while (!open.isEmpty()) {
      Label at = open.poll();
      if (at.d > maxDist) break;
      int bit = at.via.isReverse(at.node) ? 1 : 2; // isReverse(x) is true when x is the link's far end
      Integer seen = settled.get(at.via);
      if (seen != null && (seen & bit) != 0) continue;
      settled.put(at.via, seen == null ? bit : seen | bit);
      OsmNode node = at.node;
      if (!nodesCache.obtainNonHollowNode(node)) continue;
      int degree = 0;
      for (OsmLink l = node.firstlink; l != null; l = l.getNext(node)) {
        if (isRoad(l, node)) degree++;
      }
      for (OsmLink l = node.firstlink; l != null; l = l.getNext(node)) {
        if (!isRoad(l, node)) continue;
        OsmNode target = l.getTarget(node);
        if (l == at.via && degree > 1) continue; // no U-turn, except at a dead end
        if (!accessible(l, node)) continue;
        List<Integer> hits = byLink.get(l);
        if (hits != null) {
          for (int k : hits) {
            State t = targets.get(k);
            if (t.from().ilon == node.ilon && t.from().ilat == node.ilat) {
              double d = at.d + t.pos();
              if (d < road[k]) {
                road[k] = d;
                arrival[k] = at;
              }
            }
          }
        }
        double nd = at.d + length(l, node, target);
        if (nd <= maxDist) open.add(new Label(nd, target, l, at));
      }
    }
  }

  /**
   * A usable link out of {@code node}: tagged (not a beeline) and leading to another node. A link that was
   * partly cleared from memory returns its own source as target.
   */
  private static boolean isRoad(OsmLink l, OsmNode node) {
    OsmNode target = l.getTarget(node);
    return l.descriptionBitmap != null && target != null && target != node && !l.isLinkUnused();
  }

  private boolean accessible(OsmLink l, OsmNode from) {
    boolean reverse = l.isReverse(from);
    IdentityHashMap<OsmLink, Boolean> cache = reverse ? accessReverse : accessForward;
    Boolean ok = cache.get(l);
    if (ok == null) {
      rc.expctxWay.evaluate(reverse, l.descriptionBitmap);
      ok = rc.expctxWay.getCostfactor() < FORBIDDEN_COSTFACTOR;
      cache.put(l, ok);
    }
    return ok;
  }

  private double length(OsmLink l, OsmNode from, OsmNode to) {
    Double len = linkLengths.get(l);
    if (len == null) {
      int[][] geo = geometry(l, from, to);
      double d = 0;
      for (int i = 1; i < geo[0].length; i++) d += CheapRuler.distance(geo[0][i - 1], geo[1][i - 1], geo[0][i], geo[1][i]);
      len = d;
      linkLengths.put(l, len);
    }
    return len;
  }

  /** Positions of {@code l} from {@code from} to {@code to}, both ends included: {lons, lats}. */
  private int[][] geometry(OsmLink l, OsmNode from, OsmNode to) {
    List<int[]> pts = new ArrayList<>();
    pts.add(new int[]{from.ilon, from.ilat});
    if (l.geometry != null) {
      for (OsmTransferNode tn = rc.geometryDecoder.decodeGeometry(l.geometry, from, to, l.isReverse(from)); tn != null; tn = tn.next) {
        pts.add(new int[]{tn.ilon, tn.ilat});
      }
    }
    pts.add(new int[]{to.ilon, to.ilat});
    int[] lons = new int[pts.size()];
    int[] lats = new int[pts.size()];
    for (int i = 0; i < pts.size(); i++) {
      lons[i] = pts.get(i)[0];
      lats[i] = pts.get(i)[1];
    }
    return new int[][]{lons, lats};
  }

  // ---------------------------------------------------------------------------------------------------------------
  // result

  /** Road positions of the chain ending in {@code end}, oldest first, from its start or last turnaround. */
  private List<OsmNode> geometry(State end) {
    List<State> chain = new ArrayList<>();
    for (State s = end; s != null; s = s.prev) {
      chain.add(s);
      if (s.turned) {
        // before the turn the vehicle drove the way it is going now: the trace starts at the turn
        chain.add(s.prev.twin);
        break;
      }
    }
    Collections.reverse(chain);
    List<OsmNode> out = new ArrayList<>();
    State first = chain.get(0);
    out.add(new OsmNode(first.c.cLon, first.c.cLat));
    // effective position on the current link: a step back within the noise does not move the vehicle back
    double pos = first.pos();
    for (int i = 1; i < chain.size(); i++) {
      State s = chain.get(i - 1);
      State t = chain.get(i);
      if (t.kind == KIND_AHEAD) {
        if (t.pos() > pos) {
          appendAlong(out, t, pos, t.pos());
          pos = t.pos();
        }
      } else { // KIND_ROAD
        appendAlong(out, s, pos, s.c.length());
        List<Label> labels = new ArrayList<>();
        for (Label l = t.arrival; l != null; l = l.prev) labels.add(l);
        Collections.reverse(labels);
        for (int k = 1; k < labels.size(); k++) {
          Label from = labels.get(k - 1);
          Label to = labels.get(k);
          int[][] geo = geometry(to.via, from.node, to.node);
          for (int v = 1; v < geo[0].length; v++) add(out, geo[0][v], geo[1][v]);
        }
        appendAlong(out, t, 0., t.pos());
        pos = t.pos();
      }
    }
    return out;
  }

  /** Appends the positions of {@code s}'s link strictly after {@code fromPos} up to {@code toPos} (its projection if inside). */
  private static void appendAlong(List<OsmNode> out, State s, double fromPos, double toPos) {
    Candidate c = s.c;
    int n = c.lons.length;
    for (int k = 0; k < n; k++) {
      int i = s.forward ? k : n - 1 - k;
      double p = s.forward ? c.at[i] : c.length() - c.at[i];
      if (p > fromPos && p < toPos) add(out, c.lons[i], c.lats[i]);
    }
    if (toPos >= c.length() - 1e-6) {
      OsmNode end = s.to();
      add(out, end.ilon, end.ilat);
    } else if (Math.abs(toPos - s.pos()) < 1e-6) {
      add(out, c.cLon, c.cLat);
    }
  }

  private static void add(List<OsmNode> out, int lon, int lat) {
    OsmNode last = out.get(out.size() - 1);
    if (last.ilon != lon || last.ilat != lat) out.add(new OsmNode(lon, lat));
  }

  /** The nearest vertex of {@code best}'s link behind its projection, more than the via tolerance away; null if none. */
  private static OsmNode behindOnLink(State best) {
    double tol = RoutingContext.REVERSE_GUARD_VIA_TOLERANCE_M;
    Candidate c = best.c;
    int n = c.lons.length;
    for (int k = n - 1; k >= 0; k--) {
      int i = best.forward ? k : n - 1 - k;
      double p = best.forward ? c.at[i] : c.length() - c.at[i];
      if (p < best.pos() - tol) return new OsmNode(c.lons[i], c.lats[i]);
    }
    return null;
  }

}
