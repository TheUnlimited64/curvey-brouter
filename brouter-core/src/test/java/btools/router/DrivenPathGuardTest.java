package btools.router;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import java.io.File;
import java.net.URL;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import btools.mapaccess.OsmNode;
import btools.util.CheapRuler;

/**
 * No-U-turn guard from the path the vehicle actually drove ({@code drivenpath}), on the Dreieich test
 * segments. The engine matches the path to roads and forbids the first leg to start back along the road
 * section it arrived on: the exact rule of {@code arrivedfrom}, found by the engine itself. A start with
 * no way forward turns around instead of failing.
 *
 * <p>Scenario: the free route leaves the start toward its first step S. The vehicle arrives from S's
 * side: its driven path is the engine's own route from a point about {@link #DRIVEN_LENGTH_M} out along
 * the free route to the start, sampled like GPS fixes. Its arrival position P (last node more than 2 m
 * before the start) is what {@code arrivedfrom} would name.
 */
public class DrivenPathGuardTest {
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

  @Before
  public void before() {
    URL resulturl = this.getClass().getResource("/testtrack0.gpx");
    Assert.assertNotNull("reference result not found: ", resulturl);
    workingDir = new File(resulturl.getFile()).getParentFile();
  }

  private static final class Request {
    boolean continueStraight = true;
    List<OsmNode> drivenPath;
    List<OsmNode> arrivedFrom;
    String nogos;
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
    rctx.drivenPath = r.drivenPath;
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

  /** The engine's first step: the first track node more than 2 m from the start. */
  private static OsmPathElement firstStep(OsmTrack t) {
    OsmPathElement s = t.nodes.get(0);
    for (OsmPathElement e : t.nodes) {
      if (CheapRuler.distance(e.getILon(), e.getILat(), s.getILon(), s.getILat()) > 2) return e;
    }
    return null;
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

  /** The legal way the vehicle drove from the S side to the start (no guard). */
  private OsmTrack arrival(double[][] pair, OsmTrack free) {
    Request plain = new Request();
    plain.continueStraight = false;
    return route(new double[][]{originOnFirstStepSide(free), pair[0]}, plain).getFoundTrack();
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
        return java.util.Collections.singletonList(new OsmNode(e.getILon(), e.getILat()));
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

  /**
   * The vehicle arrived from S, drove on past the start away from S, turned around (a closure) and is back at
   * the start heading toward S. Only the part after the turn is its arrival, so the start may leave toward S:
   * the outcome is the free route's, not the one guarded against S.
   */
  @Test
  public void aTurnaroundGuardsOnlyTheWayBack() {
    int checked = 0;
    for (double[][] pair : PAIRS) {
      RoutingEngine freeRun = free(pair);
      OsmTrack free = freeRun.getFoundTrack();
      OsmTrack arrival = arrival(pair, free);
      Request forward = new Request();
      forward.drivenPath = sampled(arrival);
      RoutingEngine guarded = route(pair, forward);
      if (guarded.getFoundTrack() == null || guarded.getFoundTrack().distance == free.distance) continue;
      // beyond the start, away from S, and back: close enough that every position is within the match corridor
      OsmTrack away = guarded.getFoundTrack();
      List<OsmNode> path = new ArrayList<>(forward.drivenPath);
      path.add(along(away, 12.));
      path.add(along(away, 25.));
      path.add(along(away, 12.));
      Request turned = new Request();
      turned.drivenPath = path;

      assertSameOutcome("pair " + pair[0][0], freeRun, route(pair, turned));
      checked++;
    }
    Assert.assertTrue("premise: some pair is guarded away from the free route", checked > 0);
  }

  /**
   * The vehicle's arrival began only a few metres back, on the same stretch of road as the start (no road
   * node in between). The arrival position is then mid-road, not a node the first step ends at; the guard
   * must still recognise a step toward it as going back. (An equality test on the step's end node never fires.)
   * The arrival is an engine route (legal on one-way roads), like in the other tests.
   */
  @Test
  public void anArrivalWithinOneStretchOfRoadStillGuards() {
    int guarded = 0;
    for (double[][] pair : PAIRS) {
      OsmTrack free = free(pair).getFoundTrack();
      OsmNode freeFirst = step(free);
      for (double back : new double[]{15., 25., 40., 80.}) {
        OsmNode originNode = along(free, back);
        double[] origin = {originNode.ilat / 1e6 - 90., originNode.ilon / 1e6 - 180.};
        Request plain = new Request();
        plain.continueStraight = false;
        OsmTrack arrival = route(new double[][]{origin, pair[0]}, plain).getFoundTrack();
        if (arrival == null || arrival.nodes.size() < 2) continue;
        OsmPathElement end = arrival.nodes.get(arrival.nodes.size() - 1);
        OsmNode arrivalP = null;
        for (int k = arrival.nodes.size() - 1; k >= 0 && arrivalP == null; k--) {
          OsmPathElement e = arrival.nodes.get(k);
          if (CheapRuler.distance(e.getILon(), e.getILat(), end.getILon(), end.getILat()) > 2) arrivalP = new OsmNode(e.getILon(), e.getILat());
        }
        if (arrivalP == null) continue;
        // premise: the free route starts back along the arrival
        if (angle(bearing(new OsmNode(end.getILon(), end.getILat()), freeFirst), bearing(new OsmNode(end.getILon(), end.getILat()), arrivalP)) > 20.) continue;

        List<OsmNode> tail = new ArrayList<>();
        for (OsmPathElement e : arrival.nodes) tail.add(new OsmNode(e.getILon(), e.getILat()));
        Request exact = new Request();
        exact.arrivedFrom = tail;
        if (route(pair, exact).getFoundTrack() == null) continue; // dead end: covered elsewhere
        Request driven = new Request();
        driven.drivenPath = everyMetres(arrival, 6.);

        for (Request r : new Request[]{exact, driven}) {
          RoutingEngine run = route(pair, r);
          Assert.assertNotNull(run.getErrorMessage(), run.getFoundTrack());
          OsmNode first = step(run.getFoundTrack());
          Assert.assertTrue("pair " + pair[0][0] + " back " + back + (r == exact ? " arrivedfrom" : " drivenpath")
            + " still starts back along the arrival", first.ilon != freeFirst.ilon || first.ilat != freeFirst.ilat);
          guarded++;
        }
      }
    }
    Assert.assertTrue("premise: the free route goes back toward some arrival", guarded > 0);
  }

  /**
   * Start and arrival origin both in the middle of one long straight segment of the road (no road node between
   * them and the start), the target behind the arrival. The arrival position is a mid-segment point, never the
   * node the first step ends at: the guard must still see a step toward it as going back.
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
        Request plain = new Request();
        plain.continueStraight = false;
        OsmTrack arrival = route(new double[][]{origin, x}, plain).getFoundTrack();
        OsmTrack back = route(leg, new Request()).getFoundTrack();
        if (arrival == null || arrival.nodes.size() < 2 || back == null || back.nodes.size() < 2) continue;

        List<OsmNode> tail = new ArrayList<>();
        for (OsmPathElement e : arrival.nodes) tail.add(new OsmNode(e.getILon(), e.getILat()));
        Request exact = new Request();
        exact.arrivedFrom = tail;
        Request driven = new Request();
        driven.drivenPath = everyMetres(arrival, 6.);
        OsmNode freeFirst = step(back);
        for (Request r : new Request[]{exact, driven}) {
          RoutingEngine run = route(leg, r);
          if (run.getFoundTrack() == null) continue; // no way forward (dead end): covered elsewhere
          OsmNode first = step(run.getFoundTrack());
          Assert.assertTrue("pair " + pair[0][0] + " segment " + k + (r == exact ? " arrivedfrom" : " drivenpath")
            + " still starts back along the arrival", first.ilon != freeFirst.ilon || first.ilat != freeFirst.ilat);
          guarded++;
        }
      }
    }
    Assert.assertTrue("premise: some long segment lets the free route go back", guarded > 0);
  }

  /** The point {@code f} of the way from a to b, as {lat, lon}. */
  private static double[] lerp(OsmPathElement a, OsmPathElement b, double f) {
    return new double[]{
      (a.getILat() + f * (b.getILat() - a.getILat())) / 1e6 - 90.,
      (a.getILon() + f * (b.getILon() - a.getILon())) / 1e6 - 180.};
  }

  /** Positions every {@code spacing} metres along the track (interpolated, like GPS fixes), oldest first, without the end. */
  private static List<OsmNode> everyMetres(OsmTrack t, double spacing) {
    List<OsmNode> out = new ArrayList<>();
    double total = 0;
    for (int k = 1; k < t.nodes.size(); k++) total += segment(t, k);
    for (double at = 0; at < total - 2.; at += spacing) {
      double left = at;
      for (int k = 1; k < t.nodes.size(); k++) {
        double len = segment(t, k);
        if (left <= len && len > 0) {
          OsmPathElement a = t.nodes.get(k - 1);
          OsmPathElement b = t.nodes.get(k);
          double f = left / len;
          out.add(new OsmNode(a.getILon() + (int) Math.round(f * (b.getILon() - a.getILon())),
            a.getILat() + (int) Math.round(f * (b.getILat() - a.getILat()))));
          break;
        }
        left -= len;
      }
    }
    return out;
  }

  private static double segment(OsmTrack t, int k) {
    OsmPathElement a = t.nodes.get(k - 1);
    OsmPathElement b = t.nodes.get(k);
    return CheapRuler.distance(a.getILon(), a.getILat(), b.getILon(), b.getILat());
  }

  private static OsmNode step(OsmTrack t) {
    OsmPathElement e = firstStep(t);
    return new OsmNode(e.getILon(), e.getILat());
  }

  private static double angle(double a, double b) {
    double d = Math.abs(a - b) % 360.;
    return d > 180. ? 360. - d : d;
  }

  /** One GPS point cannot tell the road from a neighbouring one: no guard, the free route. */
  @Test
  public void aSingleDrivenPositionArmsNoGuard() {
    for (double[][] pair : PAIRS) {
      RoutingEngine freeRun = free(pair);
      OsmTrack arrival = arrival(pair, freeRun.getFoundTrack());
      List<OsmNode> all = sampled(arrival);
      Request one = new Request();
      one.drivenPath = all.subList(all.size() - 1, all.size());

      assertSameOutcome("pair " + pair[0][0], freeRun, route(pair, one));
    }
  }

  /**
   * A bad oldest position (a GPS outlier, or snapped onto another road) must not disable the guard when the
   * newer positions name the arrival road.
   */
  @Test
  public void aBadOldestPositionStillGuardsFromTheNewerOnes() {
    int checked = 0;
    for (double[][] pair : PAIRS) {
      OsmTrack free = free(pair).getFoundTrack();
      OsmTrack arrival = arrival(pair, free);
      List<OsmNode> good = sampled(arrival);
      if (good.size() < 3) continue;
      Request base = new Request();
      base.drivenPath = good;
      RoutingEngine expected = route(pair, base);

      // oldest position on the road past the start: its way to the start does not pass the other positions
      OsmTrack away = expected.getFoundTrack();
      if (away == null || away.distance == free.distance) continue;
      List<OsmNode> withOutlier = new ArrayList<>(good);
      withOutlier.set(0, along(away, 100.));
      Request bad = new Request();
      bad.drivenPath = withOutlier;

      assertSameOutcome("pair " + pair[0][0], expected, route(pair, bad));
      checked++;
    }
    Assert.assertTrue("premise: some pair has a long enough arrival", checked > 0);
  }

  @Test
  public void drivenPathGuardsExactlyLikeArrivedFrom() {
    int changed = 0;
    for (double[][] pair : PAIRS) {
      OsmTrack free = free(pair).getFoundTrack();
      OsmTrack arrival = arrival(pair, free);

      Request driven = new Request();
      driven.drivenPath = sampled(arrival);
      Request exact = new Request();
      exact.arrivedFrom = arrivalPosition(arrival);
      RoutingEngine byPath = route(pair, driven);
      RoutingEngine byPosition = route(pair, exact);

      if (byPosition.getErrorMessage() != null) {
        // no way forward: the exact rule fails, the vehicle turns around instead
        Assert.assertTrue(byPosition.getErrorMessage(), byPosition.getErrorMessage().contains("without a U-turn"));
        Assert.assertNull(byPath.getErrorMessage(), byPath.getErrorMessage());
        continue;
      }
      assertSameOutcome("pair " + pair[0][0], byPosition, byPath);
      if (byPath.getFoundTrack().distance != free.distance) changed++;
    }
    Assert.assertTrue("premise: the guard changes some route", changed > 0);
  }

  @Test
  public void gpsNoiseOnTheDrivenPathGivesTheSameGuard() {
    for (double[][] pair : PAIRS) {
      OsmTrack free = free(pair).getFoundTrack();
      List<OsmNode> clean = sampled(arrival(pair, free));
      Request a = new Request();
      a.drivenPath = clean;
      Request b = new Request();
      b.drivenPath = jitter(clean, 8.);

      assertSameOutcome("pair " + pair[0][0], route(pair, a), route(pair, b));
    }
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

    Request driven = new Request();
    driven.drivenPath = sampled(arrival);
    driven.nogos = nogoAhead(arrival);
    RoutingEngine turned = route(pair, driven);

    Assert.assertNull("no way forward must not leave the vehicle without a route", turned.getErrorMessage());
    OsmPathElement first = firstStep(turned.getFoundTrack());
    Assert.assertTrue("the only way is back the way it came", first.getILon() == back.ilon && first.getILat() == back.ilat);
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
  public void aGpsJumpOffTheMatchedRoadArmsNoGuard() {
    // the newest fix 60 m off the road (a GPS jump, or the vehicle came another way): every suffix of the path
    // contains it, so the match is not trusted. (A jump further back leaves a clean newer part: see
    // aBadOldestPositionStillGuardsFromTheNewerOnes.)
    int meaningful = 0;
    for (double[][] pair : PAIRS) {
      OsmTrack free = free(pair).getFoundTrack();
      List<OsmNode> path = new ArrayList<>(sampled(arrival(pair, free)));
      Assert.assertTrue("premise: a fix to move", path.size() >= 2);
      int mid = path.size() - 1;
      OsmNode p = path.get(mid);
      OsmNode before = path.get(Math.max(0, mid - 1));
      OsmNode after = mid + 1 < path.size() ? path.get(mid + 1) : new OsmNode(free.nodes.get(0).getILon(), free.nodes.get(0).getILat());
      // sideways to the road, not along it
      int[] jumped = CheapRuler.destination(p.ilon, p.ilat, 60., (bearing(before, after) + 90.) % 360.);
      path.set(mid, new OsmNode(jumped[0], jumped[1]));
      Request r = new Request();
      r.drivenPath = path;

      assertSameOutcome("pair " + pair[0][0], free(pair), route(pair, r));
      Request clean = new Request();
      clean.drivenPath = sampled(arrival(pair, free));
      RoutingEngine guarded = route(pair, clean);
      if (guarded.getFoundTrack() != null && guarded.getFoundTrack().distance != free.distance) meaningful++;
    }
    Assert.assertTrue("premise: the clean path changes some route", meaningful > 0);
  }

  @Test
  public void withoutContinueStraightTheDrivenPathIsIgnored() {
    double[][] pair = PAIRS[0];
    Request plain = new Request();
    plain.continueStraight = false;
    RoutingEngine free = route(pair, plain);
    Request withPath = new Request();
    withPath.continueStraight = false;
    withPath.drivenPath = sampled(arrival(pair, free.getFoundTrack()));

    assertSameOutcome("no continueStraight", free, route(pair, withPath));
  }

  @Test
  public void aDrivenPathAtTheStartArmsNoGuard() {
    double[][] pair = PAIRS[0];
    RoutingEngine free = free(pair);
    OsmPathElement start = free.getFoundTrack().nodes.get(0);
    int[] near = CheapRuler.destination(start.getILon(), start.getILat(), 1., 90.);
    Request r = new Request();
    r.drivenPath = java.util.Collections.singletonList(new OsmNode(near[0], near[1]));

    assertSameOutcome("stationary", free, route(pair, r));
  }

  @Test
  public void arrivedFromTakesPrecedenceOverTheDrivenPath() {
    double[][] pair = PAIRS[0];
    OsmTrack free = free(pair).getFoundTrack();
    OsmPathElement step = firstStep(free);
    Request exact = new Request();
    exact.arrivedFrom = java.util.Collections.singletonList(new OsmNode(step.getILon(), step.getILat()));
    Request both = new Request();
    both.arrivedFrom = exact.arrivedFrom;
    // a path from the other side would guard the opposite direction
    both.drivenPath = java.util.Collections.singletonList(new OsmNode(free.nodes.get(free.nodes.size() - 1).getILon(),
      free.nodes.get(free.nodes.size() - 1).getILat()));

    assertSameOutcome("precedence", route(pair, exact), route(pair, both));
  }
}
