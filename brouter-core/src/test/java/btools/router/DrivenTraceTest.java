package btools.router;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import java.io.File;
import java.net.URL;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import btools.mapaccess.OsmNode;
import btools.util.CheapRuler;

/**
 * The driven trace (#204, Curvey {@code docs/engine-driven-trace.md}) on the Dreieich test segments: the engine
 * reconstructs the road path the vehicle drove from its observations, starts the reroute where that path ends,
 * forbids turning on the spot (R1) and makes going back along the path expensive (R2).
 *
 * <p>Scenario: the free route leaves the start toward its first step S. The vehicle arrives from S's side: its
 * observations are the engine's own route from a point about {@link #DRIVEN_LENGTH_M} out along the free route to
 * the start, sampled like GPS fixes.
 */
public class DrivenTraceTest {
  private File workingDir;

  /** Start/target pairs (lat, lon) whose free route leaves the start along a road. */
  private static final double[][][] PAIRS = {
    {{49.997500, 8.723700}, {50.002409, 8.721041}},
    {{50.002500, 8.720900}, {49.997494, 8.723686}},
    {{50.001131, 8.716371}, {49.997500, 8.723700}},
  };

  /** How far back along the road the driven path reaches. */
  private static final double DRIVEN_LENGTH_M = 150.;
  /** Spacing of the driven path's points, like GPS fixes. */
  private static final double DRIVEN_SPACING_M = 25.;
  /** Sigma of a point snapped onto a road (the client's on-route observations). */
  private static final double ON_ROAD_SIGMA_M = 3.;
  /** Sigma of a plain GPS fix. */
  private static final double GPS_SIGMA_M = 10.;
  /** A way the driven road reaches only over at least this much road counts as a separate parallel way. */
  private static final double DETOUR_TO_PARALLEL_M = 60.;
  /** A heading change of at least this much within {@link #CORNER_LENGTH_M} of road counts as a corner. */
  private static final double CORNER_TURN_DEG = 80.;
  private static final double CORNER_LENGTH_M = 15.;
  /** Farther from the road than the match's smallest search radius ({@link DrivenTraceMatcher#MIN_RADIUS_M}). */
  private static final double FAR_OFF_ROAD_M = 25.;
  /** A constant sideways GPS offset the receiver does not report, about 2.3 sigma of {@link #BIASED_SIGMA_M}. */
  private static final double BIAS_M = 18.;
  private static final double BIASED_SIGMA_M = 8.;
  /** Two snapped starts this close are the same place. */
  private static final double SAME_SNAP_M = 3.;

  @Before
  public void before() {
    URL resulturl = this.getClass().getResource("/testtrack0.gpx");
    Assert.assertNotNull("reference result not found: ", resulturl);
    workingDir = new File(resulturl.getFile()).getParentFile();
  }

  private static final class Request {
    boolean continueStraight = true;
    List<TraceObservation> drivenTrace;
    List<OsmNode> arrivedFrom;
    String nogos;
    /** Profile variable reversalCostFactor; null = the engine's default. 1 switches R2 off. */
    Double reversalCostFactor;
  }

  private RoutingEngine route(double[][] latLon, Request r) {
    List<OsmNodeNamed> wplist = new ArrayList<>();
    for (int i = 0; i < latLon.length; i++) {
      OsmNodeNamed n = new OsmNodeNamed();
      n.name = i == 0 ? "from" : "to";
      n.ilat = 90000000 + (int) (latLon[i][0] * 1000000 + 0.5);
      n.ilon = 180000000 + (int) (latLon[i][1] * 1000000 + 0.5);
      wplist.add(n);
    }
    RoutingContext rctx = new RoutingContext();
    rctx.keyValues = new HashMap<>();
    if (r.continueStraight) rctx.keyValues.put("continueStraight", "1");
    if (r.reversalCostFactor != null) rctx.keyValues.put("reversalCostFactor", String.valueOf(r.reversalCostFactor));
    rctx.drivenTrace = r.drivenTrace;
    rctx.arrivedFrom = r.arrivedFrom;
    if (r.nogos != null) {
      List<OsmNodeNamed> nogoList = new RoutingParamCollector().readNogoList(r.nogos);
      RoutingContext.prepareNogoPoints(nogoList);
      rctx.nogopoints = nogoList;
    }
    rctx.localFunction = workingDir.getAbsolutePath() + "/../../../../misc/profiles2/trekking.brf";
    RoutingEngine re = new RoutingEngine(null, null,
      new File(workingDir, "/../../../../brouter-map-creator/build/resources/test/tmp/segments"), wplist, rctx);
    re.quite = true;
    re.doRun(0);
    return re;
  }

  private RoutingEngine free(double[][] pair) {
    return route(pair, new Request());
  }

  private static double[] latLon(OsmNode n) {
    return new double[]{n.ilat / 1e6 - 90., n.ilon / 1e6 - 180.};
  }

  private static OsmNode node(double[] latLon) {
    return new OsmNode(180000000 + (int) (latLon[1] * 1000000 + 0.5), 90000000 + (int) (latLon[0] * 1000000 + 0.5));
  }

  /** Observations at {@code positions} with {@code sigma}, then the start itself with {@code sigma}. */
  private static List<TraceObservation> observations(List<OsmNode> positions, double sigma, double[] start) {
    List<TraceObservation> out = new ArrayList<>();
    for (OsmNode p : positions) out.add(new TraceObservation(p.ilon, p.ilat, sigma));
    OsmNode s = node(start);
    out.add(new TraceObservation(s.ilon, s.ilat, sigma));
    return out;
  }

  private static Request traced(List<OsmNode> positions, double sigma, double[] start) {
    Request r = new Request();
    r.drivenTrace = observations(positions, sigma, start);
    return r;
  }

  /** The engine's first step: the first track node more than 2 m from the start. */
  private static OsmPathElement firstStep(OsmTrack t) {
    OsmPathElement s = t.nodes.get(0);
    for (OsmPathElement e : t.nodes) {
      if (CheapRuler.distance(e.getILon(), e.getILat(), s.getILon(), s.getILat()) > 2) return e;
    }
    return null;
  }

  private static OsmNode step(OsmTrack t) {
    OsmPathElement e = firstStep(t);
    return new OsmNode(e.getILon(), e.getILat());
  }

  private static boolean same(OsmNode a, OsmNode b) {
    return a.ilon == b.ilon && a.ilat == b.ilat;
  }

  /** The free route's node about {@link #DRIVEN_LENGTH_M} from its start: where the vehicle came from. */
  private static double[] originOnFirstStepSide(OsmTrack free) {
    OsmPathElement prev = free.nodes.get(0);
    double dist = 0;
    for (OsmPathElement e : free.nodes) {
      dist += CheapRuler.distance(prev.getILon(), prev.getILat(), e.getILon(), e.getILat());
      prev = e;
      if (dist > DRIVEN_LENGTH_M) return new double[]{e.getILat() / 1e6 - 90., e.getILon() / 1e6 - 180.};
    }
    return new double[]{prev.getILat() / 1e6 - 90., prev.getILon() / 1e6 - 180.};
  }

  private static Request plain() {
    Request r = new Request();
    r.continueStraight = false;
    return r;
  }

  /** The legal way the vehicle drove from the S side to the start (no guard). */
  private OsmTrack arrival(double[][] pair, OsmTrack free) {
    return route(new double[][]{originOnFirstStepSide(free), pair[0]}, plain()).getFoundTrack();
  }

  /** The arrival track's nodes every {@link #DRIVEN_SPACING_M}, oldest first, without the start: GPS-like fixes. */
  private static List<OsmNode> sampled(OsmTrack arrival) {
    List<OsmNode> out = new ArrayList<>();
    OsmPathElement end = arrival.nodes.get(arrival.nodes.size() - 1);
    OsmPathElement prev = arrival.nodes.get(0);
    double sinceLast = DRIVEN_SPACING_M;
    for (OsmPathElement e : arrival.nodes) {
      sinceLast += CheapRuler.distance(prev.getILon(), prev.getILat(), e.getILon(), e.getILat());
      prev = e;
      boolean atStart = CheapRuler.distance(e.getILon(), e.getILat(), end.getILon(), end.getILat()) <= 2;
      if (sinceLast >= DRIVEN_SPACING_M && !atStart) {
        out.add(new OsmNode(e.getILon(), e.getILat()));
        sinceLast = 0;
      }
    }
    return out;
  }

  /** The arrival position {@code arrivedfrom} would name: the arrival's last node more than 2 m before its end. */
  private static List<OsmNode> arrivalPosition(OsmTrack arrival) {
    OsmPathElement end = arrival.nodes.get(arrival.nodes.size() - 1);
    for (int i = arrival.nodes.size() - 1; i >= 0; i--) {
      OsmPathElement e = arrival.nodes.get(i);
      if (CheapRuler.distance(e.getILon(), e.getILat(), end.getILon(), end.getILat()) > 2) {
        return Collections.singletonList(new OsmNode(e.getILon(), e.getILat()));
      }
    }
    return null;
  }

  /** Same points shifted sideways by up to {@code meters}, alternating, like GPS noise. */
  private static List<OsmNode> jitter(List<OsmNode> path, double meters) {
    List<OsmNode> out = new ArrayList<>();
    for (int i = 0; i < path.size(); i++) {
      OsmNode p = path.get(i);
      int[] q = CheapRuler.destination(p.ilon, p.ilat, meters, i % 2 == 0 ? 45. : 225.);
      out.add(new OsmNode(q[0], q[1]));
    }
    return out;
  }

  private static void assertSameOutcome(String what, RoutingEngine expected, RoutingEngine actual) {
    Assert.assertEquals(what + ": error", expected.getErrorMessage(), actual.getErrorMessage());
    if (expected.getFoundTrack() != null) {
      Assert.assertEquals(what + ": distance", expected.getFoundTrack().distance, actual.getFoundTrack().distance);
      Assert.assertEquals(what + ": nodes", expected.getFoundTrack().nodes.size(), actual.getFoundTrack().nodes.size());
    }
  }

  private static double bearing(OsmNode from, OsmNode to) {
    double[] k = CheapRuler.getLonLatToMeterScales((from.ilat + to.ilat) >> 1);
    return (Math.toDegrees(Math.atan2((to.ilon - from.ilon) * k[0], (to.ilat - from.ilat) * k[1])) + 360.) % 360.;
  }

  private static double angle(double a, double b) {
    double d = Math.abs(a - b) % 360.;
    return d > 180. ? 360. - d : d;
  }

  private static double segment(OsmTrack t, int k) {
    OsmPathElement a = t.nodes.get(k - 1);
    OsmPathElement b = t.nodes.get(k);
    return CheapRuler.distance(a.getILon(), a.getILat(), b.getILon(), b.getILat());
  }

  private static double length(OsmTrack t) {
    double total = 0;
    for (int k = 1; k < t.nodes.size(); k++) total += segment(t, k);
    return total;
  }

  /** Position {@code meters} along {@code track} from its start, interpolated between its nodes. */
  private static OsmNode along(OsmTrack track, double meters) {
    OsmPathElement prev = track.nodes.get(0);
    double dist = 0;
    for (OsmPathElement e : track.nodes) {
      double step = CheapRuler.distance(prev.getILon(), prev.getILat(), e.getILon(), e.getILat());
      if (dist + step >= meters && step > 0) {
        double f = (meters - dist) / step;
        return new OsmNode(prev.getILon() + (int) Math.round(f * (e.getILon() - prev.getILon())),
          prev.getILat() + (int) Math.round(f * (e.getILat() - prev.getILat())));
      }
      dist += step;
      prev = e;
    }
    return new OsmNode(prev.getILon(), prev.getILat());
  }

  /** Positions every {@code spacing} metres along the track from {@code from} to {@code to} metres, interpolated. */
  private static List<OsmNode> between(OsmTrack t, double from, double to, double spacing) {
    List<OsmNode> out = new ArrayList<>();
    if (from <= to) {
      for (double at = from; at <= to + 1e-6; at += spacing) out.add(along(t, at));
    } else {
      for (double at = from; at >= to - 1e-6; at -= spacing) out.add(along(t, at));
    }
    return out;
  }

  /** Positions every {@code spacing} metres along the track (interpolated, like GPS fixes), oldest first, without the end. */
  private static List<OsmNode> everyMetres(OsmTrack t, double spacing) {
    List<OsmNode> out = new ArrayList<>();
    double total = length(t);
    for (double at = 0; at < total - 2.; at += spacing) out.add(along(t, at));
    return out;
  }

  private static List<OsmNode> positions(OsmTrack t) {
    List<OsmNode> out = new ArrayList<>();
    for (OsmPathElement e : t.nodes) out.add(new OsmNode(e.getILon(), e.getILat()));
    return out;
  }

  /** Metres of {@code route} that run backwards along {@code driven}, measured with the engine's own R2 test. */
  private static double backwardsAlong(OsmTrack route, List<OsmNode> driven) {
    DrivenTrace trace = DrivenTrace.tail(driven, RoutingContext.DRIVEN_TRACE_MAX_LENGTH_M);
    double back = 0;
    for (int k = 1; k < route.nodes.size(); k++) {
      OsmPathElement a = route.nodes.get(k - 1);
      OsmPathElement b = route.nodes.get(k);
      back += trace.backwardOverlapM(a.getILon(), a.getILat(), b.getILon(), b.getILat());
    }
    return back;
  }

  private static double[] lerp(OsmPathElement a, OsmPathElement b, double f) {
    return new double[]{
      (a.getILat() + f * (b.getILat() - a.getILat())) / 1e6 - 90.,
      (a.getILon() + f * (b.getILon() - a.getILon())) / 1e6 - 180.};
  }

  // ---------------------------------------------------------------------------------------------------------------
  // R1 at the reroute start: same guard as arrivedfrom, found from the observations

  @Test
  public void theTraceGuardsTheStartExactlyLikeArrivedFrom() {
    int changed = 0;
    for (double[][] pair : PAIRS) {
      OsmTrack free = free(pair).getFoundTrack();
      OsmTrack arrival = arrival(pair, free);

      Request driven = traced(sampled(arrival), ON_ROAD_SIGMA_M, pair[0]);
      driven.reversalCostFactor = 1.; // R1 alone: arrivedfrom with one position has no path for R2
      Request exact = new Request();
      exact.arrivedFrom = arrivalPosition(arrival);
      RoutingEngine byTrace = route(pair, driven);
      RoutingEngine byPosition = route(pair, exact);

      if (byPosition.getErrorMessage() != null) {
        // no way forward: the exact rule fails, the vehicle turns around instead
        Assert.assertTrue(byPosition.getErrorMessage(), byPosition.getErrorMessage().contains("without a U-turn"));
        Assert.assertNull(byTrace.getErrorMessage(), byTrace.getErrorMessage());
        continue;
      }
      assertSameOutcome("pair " + pair[0][0], byPosition, byTrace);
      if (byTrace.getFoundTrack().distance != free.distance) changed++;
    }
    Assert.assertTrue("premise: the guard changes some route", changed > 0);
  }

  /**
   * A start farther off the road than the match looks for roads (a tapped point, a fix in a car park, a fix that
   * claims more accuracy than it has): the route starts where the engine snaps it, so the trace ends there and
   * guards it as before.
   */
  @Test
  public void aStartFarOffTheRoadIsWhereTheEngineSnapsIt() {
    int compared = 0;
    for (double[][] pair : PAIRS) {
      OsmTrack free = free(pair).getFoundTrack();
      OsmTrack arrival = arrival(pair, free);
      OsmPathElement snapped = free.nodes.get(0);
      OsmNode s0 = new OsmNode(snapped.getILon(), snapped.getILat());
      double road = bearing(s0, step(free));
      for (double side : new double[]{90., 270.}) {
        // straight off the road, so the engine's snap lands where the start was
        double dir = (road + side) % 360.;
        int[] off = CheapRuler.destination(snapped.getILon(), snapped.getILat(), FAR_OFF_ROAD_M, dir);
        double[][] offPair = {latLon(new OsmNode(off[0], off[1])), pair[1]};
        OsmTrack offFree = free(offPair).getFoundTrack();
        // premise: the engine snaps the far start back onto the same place
        if (offFree == null || CheapRuler.distance(offFree.nodes.get(0).getILon(), offFree.nodes.get(0).getILat(),
          snapped.getILon(), snapped.getILat()) > SAME_SNAP_M) {
          continue;
        }
        Request driven = traced(sampled(arrival), ON_ROAD_SIGMA_M, offPair[0]);
        // the start claims 1 m: the match looks for roads only within its smallest radius, short of the road
        TraceObservation start = driven.drivenTrace.remove(driven.drivenTrace.size() - 1);
        driven.drivenTrace.add(new TraceObservation(start.ilon, start.ilat, TraceObservation.MIN_SIGMA_M));
        driven.reversalCostFactor = 1.;
        Request exact = new Request();
        exact.arrivedFrom = arrivalPosition(arrival);
        exact.reversalCostFactor = 1.;
        assertSameOutcome("pair " + pair[0][0] + " off " + dir, route(offPair, exact), route(offPair, driven));
        compared++;
      }
    }
    Assert.assertTrue("premise: some far start snaps back onto the arrival's end", compared > 0);
  }

  /**
   * A receiver off by a constant sideways offset (multipath, a biased fix) that it does not report: every fix lies
   * about 2 sigma beside the road. Consistently so: the road still explains them better than calling them all wrong.
   */
  @Test
  public void aConsistentlyOffsetTraceStillGuardsTheStart() {
    for (double[][] pair : PAIRS) {
      OsmTrack free = free(pair).getFoundTrack();
      OsmTrack arrival = arrival(pair, free);
      List<OsmNode> clean = sampled(arrival);
      OsmNode last = clean.get(clean.size() - 1);
      double side = (bearing(last, node(pair[0])) + 90.) % 360.;
      List<OsmNode> offset = new ArrayList<>();
      for (OsmNode p : clean) {
        int[] q = CheapRuler.destination(p.ilon, p.ilat, BIAS_M, side);
        offset.add(new OsmNode(q[0], q[1]));
      }
      Request driven = traced(offset, BIASED_SIGMA_M, pair[0]);
      driven.reversalCostFactor = 1.;
      Request exact = new Request();
      exact.arrivedFrom = arrivalPosition(arrival);
      exact.reversalCostFactor = 1.;
      assertSameOutcome("pair " + pair[0][0], route(pair, exact), route(pair, driven));
    }
  }

  @Test
  public void gpsNoiseOnTheObservationsGivesTheSameRoute() {
    for (double[][] pair : PAIRS) {
      OsmTrack free = free(pair).getFoundTrack();
      List<OsmNode> clean = sampled(arrival(pair, free));

      assertSameOutcome("pair " + pair[0][0],
        route(pair, traced(clean, ON_ROAD_SIGMA_M, pair[0])),
        route(pair, traced(jitter(clean, 8.), GPS_SIGMA_M, pair[0])));
    }
  }

  /** A GPS outlier among the observations is ignored: same outcome as without it. */
  @Test
  public void aFixFarFromTheRoadIsIgnored() {
    int checked = 0;
    for (double[][] pair : PAIRS) {
      OsmTrack free = free(pair).getFoundTrack();
      List<OsmNode> clean = everyMetres(arrival(pair, free), 12.);
      if (clean.size() < 4) continue;
      RoutingEngine expected = route(pair, traced(clean, ON_ROAD_SIGMA_M, pair[0]));
      if (expected.getFoundTrack() == null || expected.getFoundTrack().distance == free.distance) continue;

      // the newest fix jumps 45 m sideways
      List<OsmNode> noisy = new ArrayList<>(clean);
      int last = noisy.size() - 1;
      OsmNode p = noisy.get(last);
      int[] off = CheapRuler.destination(p.ilon, p.ilat, 45., (bearing(noisy.get(last - 2), noisy.get(last - 1)) + 90.) % 360.);
      noisy.set(last, new OsmNode(off[0], off[1]));

      assertSameOutcome("pair " + pair[0][0], expected, route(pair, traced(noisy, ON_ROAD_SIGMA_M, pair[0])));
      checked++;
    }
    Assert.assertTrue("premise: some pair has a guarded route", checked > 0);
  }

  /**
   * A bad oldest observation (on the road past the start, as if the vehicle came from there) is outvoted by the
   * newer ones when fixes come as often as the client records them: the start is guarded the same way. (The match
   * may keep the bad fix and put the road to it into the trace: one fix that claims 3 m on a road reachable from
   * the others is a likely explanation. That only adds cost to going back along that road; it never blocks.)
   */
  @Test
  public void aBadOldestObservationIsOutvotedByTheNewerOnes() {
    int checked = 0;
    for (double[][] pair : PAIRS) {
      OsmTrack free = free(pair).getFoundTrack();
      List<OsmNode> good = everyMetres(arrival(pair, free), 10.);
      RoutingEngine expected = route(pair, traced(good, ON_ROAD_SIGMA_M, pair[0]));
      OsmTrack away = expected.getFoundTrack();
      if (away == null || away.distance == free.distance) continue;

      List<OsmNode> withOutlier = new ArrayList<>(good);
      withOutlier.set(0, along(away, 100.));

      RoutingEngine actual = route(pair, traced(withOutlier, ON_ROAD_SIGMA_M, pair[0]));
      Assert.assertNull(actual.getErrorMessage(), actual.getErrorMessage());
      Assert.assertTrue("pair " + pair[0][0] + ": same first step as the guarded route",
        same(step(expected.getFoundTrack()), step(actual.getFoundTrack())));
      checked++;
    }
    Assert.assertTrue("premise: some pair has a guarded route", checked > 0);
  }

  /**
   * With only a few sparse fixes, a bad oldest one can make the direction a tie. Then no guard is armed: the
   * route is the guarded one or the free one, never one guarded the wrong way.
   */
  @Test
  public void aBadObservationAmongFewNeverGuardsTheWrongWay() {
    for (double[][] pair : PAIRS) {
      RoutingEngine freeRun = free(pair);
      OsmTrack free = freeRun.getFoundTrack();
      List<OsmNode> good = sampled(arrival(pair, free));
      RoutingEngine expected = route(pair, traced(good, ON_ROAD_SIGMA_M, pair[0]));
      OsmTrack away = expected.getFoundTrack();
      if (away == null || away.distance == free.distance || good.size() < 2) continue;

      List<OsmNode> withOutlier = new ArrayList<>(good);
      withOutlier.set(0, along(away, 100.));
      RoutingEngine actual = route(pair, traced(withOutlier, ON_ROAD_SIGMA_M, pair[0]));

      Assert.assertNull(actual.getErrorMessage());
      int d = actual.getFoundTrack().distance;
      Assert.assertTrue("pair " + pair[0][0] + ": " + d + " m is neither the guarded (" + away.distance + ") nor the free route ("
        + free.distance + ")", d == away.distance || d == free.distance);
    }
  }

  /** One position cannot tell a direction: no guard, the free route. */
  @Test
  public void theStartAloneArmsNothing() {
    for (double[][] pair : PAIRS) {
      assertSameOutcome("pair " + pair[0][0], free(pair), route(pair, traced(Collections.emptyList(), ON_ROAD_SIGMA_M, pair[0])));
    }
  }

  @Test
  public void aStandingVehicleArmsNothing() {
    double[][] pair = PAIRS[0];
    OsmNode s = node(pair[0]);
    List<OsmNode> around = new ArrayList<>();
    for (int k = 0; k < 4; k++) {
      int[] q = CheapRuler.destination(s.ilon, s.ilat, 0.8, 90. * k);
      around.add(new OsmNode(q[0], q[1]));
    }

    assertSameOutcome("stationary", free(pair), route(pair, traced(around, GPS_SIGMA_M, pair[0])));
  }

  /** An impassable circle 12 m past the start in the arrival direction: no way forward. */
  private static String nogoAhead(OsmTrack arrival) {
    OsmNode from = arrivalPosition(arrival).get(0);
    OsmPathElement end = arrival.nodes.get(arrival.nodes.size() - 1);
    OsmNode start = new OsmNode(end.getILon(), end.getILat());
    int[] c = CheapRuler.destination(start.ilon, start.ilat, 12., bearing(from, start));
    return ((c[0] - 180000000) / 1e6) + "," + ((c[1] - 90000000) / 1e6) + ",8";
  }

  @Test
  public void aDeadEndAheadTurnsAroundInsteadOfFailing() {
    double[][] pair = PAIRS[0];
    OsmTrack arrival = arrival(pair, free(pair).getFoundTrack());
    OsmNode back = arrivalPosition(arrival).get(0);

    Request driven = traced(sampled(arrival), ON_ROAD_SIGMA_M, pair[0]);
    driven.nogos = nogoAhead(arrival);
    RoutingEngine turned = route(pair, driven);

    Assert.assertNull("no way forward must not leave the vehicle without a route", turned.getErrorMessage());
    Assert.assertTrue("the only way is back the way it came", same(step(turned.getFoundTrack()), back));
  }

  @Test
  public void aPlannedViaDeadEndStillFails() {
    // the same situation through arrivedfrom (a planned leg join) keeps the strict #200 behaviour
    double[][] pair = PAIRS[0];
    OsmTrack arrival = arrival(pair, free(pair).getFoundTrack());

    Request exact = new Request();
    exact.arrivedFrom = arrivalPosition(arrival);
    exact.nogos = nogoAhead(arrival);
    RoutingEngine blocked = route(pair, exact);

    Assert.assertNotNull(blocked.getErrorMessage());
    Assert.assertTrue(blocked.getErrorMessage(), blocked.getErrorMessage().contains("without a U-turn"));
  }

  @Test
  public void withoutContinueStraightTheTraceIsIgnored() {
    double[][] pair = PAIRS[0];
    RoutingEngine free = route(pair, plain());
    Request withTrace = traced(sampled(arrival(pair, free.getFoundTrack())), ON_ROAD_SIGMA_M, pair[0]);
    withTrace.continueStraight = false;

    assertSameOutcome("no continueStraight", free, route(pair, withTrace));
  }

  @Test
  public void arrivedFromTakesPrecedenceOverTheTrace() {
    double[][] pair = PAIRS[0];
    OsmTrack free = free(pair).getFoundTrack();
    OsmPathElement first = firstStep(free);
    Request exact = new Request();
    exact.arrivedFrom = Collections.singletonList(new OsmNode(first.getILon(), first.getILat()));
    Request both = new Request();
    both.arrivedFrom = exact.arrivedFrom;
    // a trace from the other side would guard the opposite direction
    OsmPathElement end = free.nodes.get(free.nodes.size() - 1);
    both.drivenTrace = observations(Collections.singletonList(new OsmNode(end.getILon(), end.getILat())), ON_ROAD_SIGMA_M, pair[0]);

    assertSameOutcome("precedence", route(pair, exact), route(pair, both));
  }

  /**
   * Start and arrival origin both in the middle of one long straight segment of the road (no road node between
   * them and the start), the target behind the arrival: a step toward it is still going back.
   */
  @Test
  public void anArrivalInTheMiddleOfALongSegmentStillGuards() {
    int guarded = 0;
    for (double[][] pair : PAIRS) {
      OsmTrack route = free(pair).getFoundTrack();
      for (int k = 1; k < route.nodes.size(); k++) {
        OsmPathElement a = route.nodes.get(k - 1);
        OsmPathElement b = route.nodes.get(k);
        if (segment(route, k) < 30.) continue;
        double[] aLatLon = {a.getILat() / 1e6 - 90., a.getILon() / 1e6 - 180.};
        double[] x = lerp(a, b, 0.7);
        double[] origin = lerp(a, b, 0.2);
        double[][] leg = {x, aLatLon};
        OsmTrack arrival = route(new double[][]{origin, x}, plain()).getFoundTrack();
        OsmTrack back = route(leg, new Request()).getFoundTrack();
        if (arrival == null || arrival.nodes.size() < 2 || back == null || back.nodes.size() < 2) continue;

        Request exact = new Request();
        exact.arrivedFrom = positions(arrival);
        Request driven = traced(everyMetres(arrival, 6.), ON_ROAD_SIGMA_M, x);
        OsmNode freeFirst = step(back);
        for (Request r : new Request[]{exact, driven}) {
          RoutingEngine run = route(leg, r);
          if (run.getFoundTrack() == null) continue; // no way forward (dead end): covered elsewhere
          Assert.assertFalse("pair " + pair[0][0] + " segment " + k + (r == exact ? " arrivedfrom" : " driventrace")
            + " still starts back along the arrival", same(step(run.getFoundTrack()), freeFirst));
          guarded++;
        }
      }
    }
    Assert.assertTrue("premise: some long segment lets the free route go back", guarded > 0);
  }

  // ---------------------------------------------------------------------------------------------------------------
  // what the trace adds over a single arrival position

  /**
   * The vehicle arrived from S, drove on past the start, turned around (a closure) and is back at the start heading
   * toward S. The trace is cut at the turn: the way back is now toward the turn, not toward S. Toward S is the
   * free route's way and stays allowed; a target beyond the turn may not be reached by turning on the spot.
   */
  @Test
  public void aTurnaroundInTheTraceGuardsTheWayItTurnedFrom() {
    int checked = 0;
    for (double[][] pair : PAIRS) {
      RoutingEngine freeRun = free(pair);
      OsmTrack free = freeRun.getFoundTrack();
      OsmTrack arrival = arrival(pair, free);
      // the road beyond the start (away from S): the arrival-guarded route's start
      RoutingEngine awayRun = route(pair, traced(sampled(arrival), ON_ROAD_SIGMA_M, pair[0]));
      OsmTrack away = awayRun.getFoundTrack();
      if (away == null || away.distance == free.distance || length(away) < 200.) continue;

      List<OsmNode> turned = new ArrayList<>(sampled(arrival));
      turned.addAll(between(away, 10., 60., 10.));
      turned.addAll(between(away, 50., 10., 10.));

      // toward S: the vehicle's way now, same as the free route
      RoutingEngine towardS = route(pair, traced(turned, ON_ROAD_SIGMA_M, pair[0]));
      Assert.assertNull(towardS.getErrorMessage());
      Assert.assertTrue("pair " + pair[0][0] + ": after the turn S is ahead", same(step(towardS.getFoundTrack()), step(free)));

      // a target beyond the turn: the free route turns on the spot, the traced one must not
      double[][] beyond = {pair[0], latLon(along(away, 180.))};
      OsmTrack freeBeyond = route(beyond, new Request()).getFoundTrack();
      Assert.assertTrue("premise: the free route to beyond the turn leaves away from S", same(step(freeBeyond), step(away)));
      RoutingEngine guarded = route(beyond, traced(turned, ON_ROAD_SIGMA_M, pair[0]));
      Assert.assertNull(guarded.getErrorMessage());
      Assert.assertFalse("pair " + pair[0][0] + ": turned on the spot back toward the turn",
        same(step(guarded.getFoundTrack()), step(away)));
      checked++;
    }
    Assert.assertTrue("premise: some pair has a road beyond the start", checked > 0);
  }

  /**
   * Target behind the vehicle. R1 alone stops the turn on the spot, but the route can still turn later and come back
   * along the road just driven. R2 makes that cost extra: the route goes back along the trace less, never more.
   */
  @Test
  public void goingBackAlongTheTraceLaterCostsExtra() {
    int reduced = 0;
    for (double[][] pair : PAIRS) {
      OsmTrack free = free(pair).getFoundTrack();
      OsmTrack arrival = arrival(pair, free);
      double[][] behind = {pair[0], originOnFirstStepSide(free)};

      Request r1Only = traced(sampled(arrival), ON_ROAD_SIGMA_M, pair[0]);
      r1Only.reversalCostFactor = 1.;
      Request both = traced(sampled(arrival), ON_ROAD_SIGMA_M, pair[0]);
      OsmTrack withoutR2 = route(behind, r1Only).getFoundTrack();
      OsmTrack withR2 = route(behind, both).getFoundTrack();
      Assert.assertNotNull(withoutR2);
      Assert.assertNotNull(withR2);

      double before = backwardsAlong(withoutR2, positions(arrival));
      double after = backwardsAlong(withR2, positions(arrival));
      Assert.assertTrue("pair " + pair[0][0] + ": R2 goes back more (" + after + " m) than without (" + before + " m)", after <= before + 1.);
      if (after < before - 10.) reduced++;
    }
    Assert.assertTrue("premise: some route came back along the trace without R2", reduced > 0);
  }

  /**
   * A route that goes on from the start is never changed by the trace: R1 does not fire and R2 only adds cost to
   * going back, so the free route stays the cheapest. This holds right before corners too: no angle is looked at,
   * so a corner ahead is just road.
   */
  @Test
  public void aRouteThatGoesOnIsNeverChanged() {
    int compared = 0;
    int beforeCorner = 0;
    for (double[][] pair : PAIRS) {
      OsmTrack base = free(pair).getFoundTrack();
      double total = length(base);
      double[] target = latLon(new OsmNode(base.nodes.get(base.nodes.size() - 1).getILon(), base.nodes.get(base.nodes.size() - 1).getILat()));
      for (double at = 160.; at < total - 60.; at += 17.) {
        OsmNode start = along(base, at);
        double[][] leg = {latLon(start), target};
        List<OsmNode> driven = between(base, at - 150., at - 10., 10.);
        RoutingEngine freeRun = route(leg, new Request());
        OsmTrack freeLeg = freeRun.getFoundTrack();
        if (freeLeg == null || backwardsAlong(freeLeg, between(base, at - 150., at, 2.)) > 0.) continue; // not going on
        RoutingEngine tracedRun = route(leg, traced(driven, ON_ROAD_SIGMA_M, latLon(start)));
        assertSameOutcome("pair " + pair[0][0] + " at " + (int) at, freeRun, tracedRun);
        compared++;
        if (cornerWithin(freeLeg, 40.)) beforeCorner++;
      }
    }
    Assert.assertTrue("premise: starts compared", compared > 10);
    Assert.assertTrue("premise: some start lies right before a corner", beforeCorner > 0);
  }

  /**
   * The track turns by at least {@link #CORNER_TURN_DEG} within {@link #CORNER_LENGTH_M} of road, somewhere in its first
   * {@code meters}: a street corner, taken over a few short segments.
   */
  private static boolean cornerWithin(OsmTrack t, double meters) {
    List<Double> at = new ArrayList<>();
    List<Double> heading = new ArrayList<>();
    double dist = 0;
    for (int k = 1; k < t.nodes.size() && dist < meters + CORNER_LENGTH_M; k++) {
      double len = segment(t, k);
      if (len < 0.5) continue;
      OsmPathElement a = t.nodes.get(k - 1);
      OsmPathElement b = t.nodes.get(k);
      at.add(dist);
      heading.add(bearing(new OsmNode(a.getILon(), a.getILat()), new OsmNode(b.getILon(), b.getILat())));
      dist += len;
    }
    for (int i = 0; i < heading.size(); i++) {
      if (at.get(i) > meters) break;
      for (int j = i + 1; j < heading.size() && at.get(j) - at.get(i) <= CORNER_LENGTH_M; j++) {
        if (angle(heading.get(i), heading.get(j)) >= CORNER_TURN_DEG) return true;
      }
    }
    return false;
  }

  /**
   * The start fix lies closer to a parallel way than to the road the vehicle drove (GPS error sideways), and that
   * way cannot be reached from the driven road without a detour. The engine starts on the driven road, where the
   * trace ends, not on the nearest way. (Where the nearest way is joined to the road right there, the vehicle may
   * well have turned onto it: the model weighs that, and this test does not judge it.)
   */
  @Test
  public void theStartIsPlacedOnTheDrivenRoad() {
    int checked = 0;
    for (double[][] pair : PAIRS) {
      OsmTrack base = free(pair).getFoundTrack();
      double total = length(base);
      double[] target = latLon(new OsmNode(base.nodes.get(base.nodes.size() - 1).getILon(), base.nodes.get(base.nodes.size() - 1).getILat()));
      for (double at = 160.; at < total - 60. && checked < 6; at += 9.) {
        OsmNode onRoad = along(base, at);
        double roadBearing = bearing(along(base, at - 5.), along(base, at + 5.));
        for (double side : new double[]{90., 270.}) {
          int[] off = CheapRuler.destination(onRoad.ilon, onRoad.ilat, 9., (roadBearing + side) % 360.);
          OsmNode fix = new OsmNode(off[0], off[1]);
          double[][] leg = {latLon(fix), target};
          OsmTrack plainStart = route(leg, new Request()).getFoundTrack();
          if (plainStart == null) continue;
          OsmNode plainCross = new OsmNode(plainStart.nodes.get(0).getILon(), plainStart.nodes.get(0).getILat());
          // the road driven and its continuation: with a 10 m sigma the start may lie a little further along
          List<OsmNode> drivenRoad = between(base, at - 150., Math.min(total, at + 40.), 2.);
          if (distanceTo(plainCross, drivenRoad) < 2.) continue; // the nearest road is the driven one: nothing to show
          OsmTrack connection = route(new double[][]{latLon(along(base, at - 10.)), latLon(plainCross)}, plain()).getFoundTrack();
          if (connection == null || connection.distance < DETOUR_TO_PARALLEL_M) continue; // joined right there

          List<TraceObservation> obs = new ArrayList<>();
          for (OsmNode p : between(base, at - 150., at - 10., 10.)) obs.add(new TraceObservation(p.ilon, p.ilat, ON_ROAD_SIGMA_M));
          obs.add(new TraceObservation(fix.ilon, fix.ilat, GPS_SIGMA_M));
          Request r = new Request();
          r.drivenTrace = obs;
          OsmTrack traced = route(leg, r).getFoundTrack();
          Assert.assertNotNull(traced);
          OsmNode cross = new OsmNode(traced.nodes.get(0).getILon(), traced.nodes.get(0).getILat());
          Assert.assertTrue("pair " + pair[0][0] + " at " + (int) at + ": start not on the driven road ("
            + (int) distanceTo(cross, drivenRoad) + " m off)", distanceTo(cross, drivenRoad) < 2.);
          checked++;
        }
      }
    }
    Assert.assertTrue("premise: some fix is nearer another road", checked > 0);
  }

  private static double distanceTo(OsmNode p, List<OsmNode> polyline) {
    double best = Double.MAX_VALUE;
    double[] k = CheapRuler.getLonLatToMeterScales(p.ilat);
    for (int i = 1; i < polyline.size(); i++) {
      OsmNode a = polyline.get(i - 1);
      OsmNode b = polyline.get(i);
      double ax = (a.ilon - p.ilon) * k[0], ay = (a.ilat - p.ilat) * k[1];
      double bx = (b.ilon - p.ilon) * k[0], by = (b.ilat - p.ilat) * k[1];
      double dx = bx - ax, dy = by - ay;
      double len2 = dx * dx + dy * dy;
      double t = len2 == 0 ? 0 : Math.max(0, Math.min(1, -(ax * dx + ay * dy) / len2));
      double x = ax + t * dx, y = ay + t * dy;
      best = Math.min(best, Math.sqrt(x * x + y * y));
    }
    return best;
  }

  @Test
  public void parsesTraceObservationsAndSkipsMalformedOnes() {
    List<TraceObservation> obs = RoutingParamCollector.readTraceObservations("8.7237,49.9975,3|bad|8.72,49.99|8.7238,49.9976,0|8.7239,49.9977,12.5");
    Assert.assertEquals(2, obs.size());
    Assert.assertEquals(180000000 + 8723700, obs.get(0).ilon);
    Assert.assertEquals(90000000 + 49997500, obs.get(0).ilat);
    Assert.assertEquals(3., obs.get(0).sigmaM, 0.);
    Assert.assertEquals(12.5, obs.get(1).sigmaM, 0.);
    Assert.assertNull(RoutingParamCollector.readTraceObservations("x"));
  }
}
