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
 * Curvey #466 (design study): a reroute that comes back onto the planned route. On the Dreieich test segments with
 * {@code car-vario}: the plan's rest goes as {@code referencetrack}, {@code referencejoin} lets the first leg start off
 * it (where the rider is) and join it, and {@code avoidtrack} is the stretch just past where the rider left the plan.
 * Waypoints are {ilon, ilat}, the pairs of {@link ReferenceTrackRoutingTest}.
 */
public class RerouteRejoinRoutingTest {
  private File workingDir;
  private String profile;

  private static final double OFF_ROAD_M = 30.;
  private static final double SAMPLE_STEP_M = 5.;

  private static final int[][][] PAIRS = {
    {{188692849, 139999156}, {188714957, 140009349}},
    {{188707294, 140005610}, {188695495, 139994204}},
    {{188691809, 140009097}, {188722353, 140008846}},
    {{188697437, 140001302}, {188708205, 139992854}},
  };

  @Before
  public void before() {
    URL resulturl = this.getClass().getResource("/testtrack0.gpx");
    Assert.assertNotNull("reference result not found: ", resulturl);
    workingDir = new File(resulturl.getFile()).getParentFile();
    profile = workingDir.getAbsolutePath() + "/../../../../misc/profiles2/car-vario.brf";
  }

  private OsmTrack route(int[] from, int[] to, List<OsmNode> line, boolean join, List<OsmNode> avoid, String avoidFactor) {
    return route(from, to, line, join, avoid, avoidFactor, false);
  }

  private OsmTrack route(int[] from, int[] to, List<OsmNode> line, boolean join, List<OsmNode> avoid, String avoidFactor,
                         boolean closed) {
    List<OsmNodeNamed> wplist = new ArrayList<>();
    for (int[] p : new int[][]{from, to}) {
      OsmNodeNamed n = new OsmNodeNamed();
      n.name = wplist.isEmpty() ? "from" : "to";
      n.ilon = p[0];
      n.ilat = p[1];
      wplist.add(n);
    }
    RoutingContext rctx = new RoutingContext();
    rctx.keyValues = new HashMap<>();
    if (avoidFactor != null) rctx.keyValues.put("avoidedTrackCostFactor", avoidFactor);
    if (closed) rctx.keyValues.put("avoidedTrackClosed", "1");
    rctx.referenceTrackPositions = line;
    rctx.referenceJoin = join;
    rctx.avoidTrackPositions = avoid;
    rctx.localFunction = profile;
    RoutingEngine re = new RoutingEngine(null, null,
      new File(workingDir, "/../../../../brouter-map-creator/build/resources/test/tmp/segments"), wplist, rctx);
    re.quite = true;
    re.doRun(0);
    Assert.assertNull(re.getErrorMessage(), re.getErrorMessage());
    return re.getFoundTrack();
  }

  private OsmTrack plain(int[] from, int[] to) {
    return route(from, to, null, false, null, null);
  }

  private static List<OsmNode> nodes(OsmTrack track, int from, int to) {
    List<OsmNode> line = new ArrayList<>();
    for (int i = from; i < to; i++) line.add(new OsmNode(track.nodes.get(i).getILon(), track.nodes.get(i).getILat()));
    return line;
  }

  private static String ids(OsmTrack track) {
    StringBuilder sb = new StringBuilder();
    for (OsmPathElement e : track.nodes) sb.append(e.getILon()).append(',').append(e.getILat()).append(' ');
    return sb.toString();
  }

  /** Metres of {@code route} farther than {@link #OFF_ROAD_M} from {@code old}. */
  private static double offM(OsmTrack route, OsmTrack old) {
    double off = 0.;
    for (int i = 1; i < route.nodes.size(); i++) {
      OsmPathElement a = route.nodes.get(i - 1);
      OsmPathElement b = route.nodes.get(i);
      double len = CheapRuler.distance(a.getILon(), a.getILat(), b.getILon(), b.getILat());
      int samples = Math.max(1, (int) Math.ceil(len / SAMPLE_STEP_M));
      for (int s = 0; s < samples; s++) {
        double t = (s + 0.5) / samples;
        int lon = (int) (a.getILon() + t * (b.getILon() - a.getILon()));
        int lat = (int) (a.getILat() + t * (b.getILat() - a.getILat()));
        if (distanceM(old, lon, lat) > OFF_ROAD_M) off += len / samples;
      }
    }
    return off;
  }

  private static double distanceM(OsmTrack track, int lon, int lat) {
    double best = Double.MAX_VALUE;
    double[] k = CheapRuler.getLonLatToMeterScales(lat);
    for (int i = 1; i < track.nodes.size(); i++) {
      OsmPathElement a = track.nodes.get(i - 1);
      OsmPathElement b = track.nodes.get(i);
      double ux = (b.getILon() - a.getILon()) * k[0];
      double uy = (b.getILat() - a.getILat()) * k[1];
      double px = (lon - a.getILon()) * k[0];
      double py = (lat - a.getILat()) * k[1];
      double l2 = ux * ux + uy * uy;
      double t = l2 == 0. ? 0. : Math.max(0., Math.min(1., (px * ux + py * uy) / l2));
      best = Math.min(best, Math.hypot(px - t * ux, py - t * uy));
    }
    return best;
  }

  /** Metres of {@code route} that run along {@code stretch}, either way. */
  private static double alongM(OsmTrack route, List<OsmNode> stretch) {
    ReferenceTrack line = ReferenceTrack.of(stretch);
    double along = 0.;
    for (int i = 1; i < route.nodes.size(); i++) {
      OsmPathElement a = route.nodes.get(i - 1);
      OsmPathElement b = route.nodes.get(i);
      along += line.alongEitherWayM(a.getILon(), a.getILat(), b.getILon(), b.getILat());
    }
    return along;
  }

  /**
   * The rider is at the start of another pair, off the plan A→B, and rerouted to B. Without {@code referencejoin} the
   * line is ignored (#458: a leg must start on it); with it the route comes back onto the plan and rides it to B.
   */
  @Test
  public void aRerouteFromOffThePlanJoinsItOnlyWhenAsked() {
    int better = 0;
    for (int k = 0; k < PAIRS.length; k++) {
      OsmTrack old = plain(PAIRS[k][0], PAIRS[k][1]);
      int[] rider = PAIRS[(k + 2) % PAIRS.length][0];
      List<OsmNode> line = nodes(old, 0, old.nodes.size());
      OsmTrack free = plain(rider, PAIRS[k][1]);

      Assert.assertEquals(ids(free), ids(route(rider, PAIRS[k][1], line, false, null, null)));

      OsmTrack joined = route(rider, PAIRS[k][1], line, true, null, null);
      double freeOff = offM(free, old);
      double joinedOff = offM(joined, old);
      Assert.assertTrue("joined " + joinedOff + " m off the plan vs free " + freeOff + " m", joinedOff <= freeOff + SAMPLE_STEP_M);
      if (joinedOff < freeOff - 100.) better++;
    }
    Assert.assertTrue("premise: the line changes at least one reroute", better > 0);
  }

  /** The stretch past the departure costs extra either way: a route keeps off it where another way exists. */
  @Test
  public void aStretchToAvoidIsLeftWhereAnotherWayExists() {
    int avoided = 0;
    for (int[][] pair : PAIRS) {
      OsmTrack old = plain(pair[0], pair[1]);
      int n = old.nodes.size();
      List<OsmNode> stretch = nodes(old, n / 3, 2 * n / 3);

      OsmTrack around = route(pair[0], pair[1], null, false, stretch, null);

      double before = alongM(old, stretch);
      double after = alongM(around, stretch);
      Assert.assertTrue("along the stretch " + after + " m vs " + before + " m before", after <= before + SAMPLE_STEP_M);
      if (after < before / 2.) avoided++;
    }
    Assert.assertTrue("premise: some stretch has another way round", avoided > 0);
  }

  /**
   * A closed stretch is not ridden at all where another way exists; where none does (the whole route is the stretch),
   * the leg is searched again without it and still gets a route.
   */
  @Test
  public void aClosedStretchIsNotRiddenAndALegWithoutAnotherWayStillGetsARoute() {
    int closedOff = 0;
    for (int[][] pair : PAIRS) {
      OsmTrack old = plain(pair[0], pair[1]);
      int n = old.nodes.size();
      List<OsmNode> stretch = nodes(old, n / 3, 2 * n / 3);

      OsmTrack costly = route(pair[0], pair[1], null, false, stretch, null);
      OsmTrack closed = route(pair[0], pair[1], null, false, stretch, null, true);

      double costlyAlong = alongM(costly, stretch);
      double closedAlong = alongM(closed, stretch);
      Assert.assertTrue("closed " + closedAlong + " m vs costly " + costlyAlong + " m", closedAlong <= costlyAlong + SAMPLE_STEP_M);
      if (closedAlong < SAMPLE_STEP_M) closedOff++;
    }
    Assert.assertTrue("premise: some closed stretch has another way round", closedOff > 0);

    int[][] pair = PAIRS[0];
    OsmTrack old = plain(pair[0], pair[1]);
    OsmTrack whole = route(pair[0], pair[1], null, false, nodes(old, 0, old.nodes.size()), null, true);
    Assert.assertTrue(whole.nodes.size() > 1);
  }

  @Test
  public void anAvoidFactorOfOneRoutesAsWithoutTheStretch() {
    int[][] pair = PAIRS[0];
    OsmTrack old = plain(pair[0], pair[1]);
    List<OsmNode> stretch = nodes(old, old.nodes.size() / 3, 2 * old.nodes.size() / 3);

    Assert.assertEquals(ids(old), ids(route(pair[0], pair[1], null, false, stretch, "1")));
  }
}
