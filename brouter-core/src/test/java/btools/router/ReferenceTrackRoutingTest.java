package btools.router;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.File;
import java.io.IOException;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import btools.mapaccess.OsmNode;
import btools.util.CheapRuler;

/**
 * Curvey #458: a route along a reference track ({@code referencetrack}), on the Dreieich test segments with
 * {@code car-vario}. Reverse in Curvey sends the old route played backwards: the reversed route then keeps to the
 * old roads, and leaves them only where a one-way street or a turn restriction makes it.
 * Waypoints are {ilon, ilat}; every pair was picked from random pairs in the sample area.
 */
public class ReferenceTrackRoutingTest {
  private File workingDir;

  /** Same corridor as Curvey's {@code RouteDivergence.SAME_ROAD_CORRIDOR_M}: farther than this is another road. */
  private static final double OFF_ROAD_M = 30.;

  /** Sample step of {@link #offM}: fine against the 30 m corridor. */
  private static final double SAMPLE_STEP_M = 5.;

  /** Rounding along a shared road: the reversed route is on the old one, give or take a sample. */
  private static final double ON_THE_OLD_ROADS_M = 1.;

  /** Pairs without one-way streets between them whose plain reverse takes other roads. */
  private static final int[][][] TWO_WAY_PAIRS = {
    {{188692849, 139999156}, {188714957, 140009349}},
    {{188707294, 140005610}, {188695495, 139994204}},
    {{188691809, 140009097}, {188722353, 140008846}},
    {{188697437, 140001302}, {188708205, 139992854}},
  };

  /** Pairs whose old route runs along a one-way street: the reverse must leave it there. */
  private static final int[][][] ONE_WAY_PAIRS = {
    {{188733883, 139999763}, {188710374, 140003800}},
    {{188728629, 140002333}, {188710763, 140003608}},
    {{188707008, 139991203}, {188724490, 140008076}},
  };

  /**
   * Loops (start, two stops, start) whose plain reverse takes other roads, routed with the no-U-turn rule at their
   * stops ({@code continueStraight}) as Curvey routes them.
   */
  private static final int[][][] LOOPS = {
    {{188717651, 139997556}, {188694820, 139997978}, {188691447, 140000842}, {188717651, 139997556}},
    {{188715628, 139994604}, {188712294, 140001322}, {188693133, 140006117}, {188715628, 139994604}},
  };

  /**
   * Loops whose reversed legs fail BRouter's detailed re-track along the track at a high factor
   * ({@code referenceDeviationCostFactor}), found by a scan of random loops in the sample area: the first fails with
   * the reversal cost (R2) and succeeds along the track without it, the second fails along the track alone too.
   */
  private static final int[][] FAILS_WITH_R2 =
    {{188724245, 139994730}, {188707939, 140005980}, {188707105, 139995761}, {188724245, 139994730}};
  private static final int[][] FAILS_ALONG_THE_TRACK =
    {{188701811, 139995019}, {188705909, 140008236}, {188718669, 140003705}, {188701811, 139995019}};

  /** {@code continueStraight=1} for the next requests: the no-U-turn rule at stops. */
  private boolean continueStraight;

  /** The profile the next requests use; the engine writes its log to {@code debug.txt} next to its directory. */
  private String profile;

  @Rule
  public TemporaryFolder temp = new TemporaryFolder();

  @Before
  public void before() {
    URL resulturl = this.getClass().getResource("/testtrack0.gpx");
    Assert.assertNotNull("reference result not found: ", resulturl);
    workingDir = new File(resulturl.getFile()).getParentFile();
    profile = workingDir.getAbsolutePath() + "/../../../../misc/profiles2/car-vario.brf";
  }

  /** Routes from here on use a copy of the profile whose engine log goes to the returned file. */
  private File logToFile() throws IOException {
    File base = temp.newFolder();
    File profiles = new File(base, "profiles2");
    Assert.assertTrue(profiles.mkdir());
    File source = new File(profile).getParentFile();
    for (String name : new String[]{"car-vario.brf", "lookups.dat"}) {
      Files.copy(new File(source, name).toPath(), new File(profiles, name).toPath());
    }
    File log = new File(base, "debug.txt");
    Assert.assertTrue(log.createNewFile());
    profile = new File(profiles, "car-vario.brf").getAbsolutePath();
    return log;
  }

  private RoutingEngine engine(int[][] lonLat, List<OsmNode> reference, String factor) {
    List<OsmNodeNamed> wplist = new ArrayList<>();
    for (int i = 0; i < lonLat.length; i++) {
      OsmNodeNamed n = new OsmNodeNamed();
      n.name = i == 0 ? "from" : i == lonLat.length - 1 ? "to" : "via" + i;
      n.ilon = lonLat[i][0];
      n.ilat = lonLat[i][1];
      wplist.add(n);
    }
    RoutingContext rctx = new RoutingContext();
    rctx.keyValues = new HashMap<>();
    if (factor != null) rctx.keyValues.put("referenceDeviationCostFactor", factor);
    if (continueStraight) rctx.keyValues.put("continueStraight", "1");
    rctx.referenceTrackPositions = reference;
    rctx.localFunction = profile;
    RoutingEngine re = new RoutingEngine(null, null,
      new File(workingDir, "/../../../../brouter-map-creator/build/resources/test/tmp/segments"), wplist, rctx);
    re.quite = true;
    re.doRun(0);
    return re;
  }

  private OsmTrack route(int[][] lonLat, List<OsmNode> reference, String factor) {
    RoutingEngine re = engine(lonLat, reference, factor);
    Assert.assertNull(re.getErrorMessage(), re.getErrorMessage());
    return re.getFoundTrack();
  }

  private OsmTrack route(int[][] lonLat, List<OsmNode> reference) {
    return route(lonLat, reference, null);
  }

  private static int[][] reversed(int[][] lonLat) {
    int[][] r = new int[lonLat.length][];
    for (int i = 0; i < lonLat.length; i++) r[i] = lonLat[lonLat.length - 1 - i];
    return r;
  }

  /** {@code track} played backwards: what Curvey sends on Reverse. */
  private static List<OsmNode> backwards(OsmTrack track) {
    List<OsmNode> line = new ArrayList<>();
    for (int i = track.nodes.size() - 1; i >= 0; i--) {
      line.add(new OsmNode(track.nodes.get(i).getILon(), track.nodes.get(i).getILat()));
    }
    return line;
  }

  /** Metres of {@code route} farther than {@link #OFF_ROAD_M} from {@code old}, sampled every {@link #SAMPLE_STEP_M}. */
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

  private static String nodes(OsmTrack track) {
    StringBuilder sb = new StringBuilder();
    for (OsmPathElement e : track.nodes) sb.append(e.getILon()).append(',').append(e.getILat()).append(' ');
    return sb.toString();
  }

  @Test
  public void withoutAOneWayStreetTheReverseRidesTheOldRoadsTheOtherWayRound() {
    for (int[][] pair : TWO_WAY_PAIRS) {
      OsmTrack old = route(pair, null);
      Assert.assertTrue("premise: the plain reverse takes other roads", offM(route(reversed(pair), null), old) > 300.);

      OsmTrack along = route(reversed(pair), backwards(old));

      Assert.assertEquals(0., offM(along, old), ON_THE_OLD_ROADS_M);
      Assert.assertEquals(0., offM(old, along), ON_THE_OLD_ROADS_M);
    }
  }

  @Test
  public void aLoopRidesItsRoadsTheOtherWayRound() {
    continueStraight = true;
    for (int[][] loop : LOOPS) {
      OsmTrack old = route(loop, null);
      Assert.assertTrue("premise: the plain reverse takes other roads", offM(route(reversed(loop), null), old) > 300.);

      OsmTrack along = route(reversed(loop), backwards(old));

      Assert.assertEquals(0., offM(along, old), ON_THE_OLD_ROADS_M);
    }
  }

  @Test
  public void aOneWayStreetIsLeftOnlyForAShortDeviation() {
    for (int[][] pair : ONE_WAY_PAIRS) {
      OsmTrack old = route(pair, null);
      double plainOff = offM(route(reversed(pair), null), old);

      double alongOff = offM(route(reversed(pair), backwards(old)), old);

      Assert.assertTrue("along " + alongOff + " m off vs plain " + plainOff + " m", alongOff < plainOff / 2.);
    }
  }

  @Test
  public void aLegThatFailsWithTheReversalCostIsSearchedAgainAlongTheTrackWithoutIt() throws IOException {
    continueStraight = true;
    OsmTrack old = route(FAILS_WITH_R2, null);
    File log = logToFile();

    route(reversed(FAILS_WITH_R2), backwards(old), "1000");

    String text = new String(Files.readAllBytes(log.toPath()), StandardCharsets.UTF_8);
    Assert.assertTrue(text, text.contains("searching along it without the reversal cost"));
    Assert.assertFalse(text, text.contains("search failed along the reference track"));
  }

  @Test
  public void aLegThatFailsAlongTheTrackIsStillRouted() throws IOException {
    continueStraight = true;
    OsmTrack old = route(FAILS_ALONG_THE_TRACK, null);
    File log = logToFile();

    route(reversed(FAILS_ALONG_THE_TRACK), backwards(old), "100000");

    String text = new String(Files.readAllBytes(log.toPath()), StandardCharsets.UTF_8);
    Assert.assertTrue(text, text.contains("search failed along the reference track"));
  }

  @Test
  public void aFactorOfOneRoutesAsWithoutTheTrack() {
    int[][] pair = reversed(TWO_WAY_PAIRS[0]);
    OsmTrack old = route(TWO_WAY_PAIRS[0], null);

    Assert.assertEquals(nodes(route(pair, null)), nodes(route(pair, backwards(old), "1")));
  }

  /** "Current location as start": the leg from the live position to the old destination runs against the line. */
  @Test
  public void aLegAgainstTheTrackIsRoutedAsWithoutIt() throws IOException {
    for (int[][] pair : TWO_WAY_PAIRS) {
      List<OsmNode> line = backwards(route(pair, null));

      Assert.assertEquals(nodes(route(pair, null)), nodes(route(pair, line)));
    }
    File log = logToFile();
    route(TWO_WAY_PAIRS[0], backwards(route(TWO_WAY_PAIRS[0], null)));
    String text = new String(Files.readAllBytes(log.toPath()), StandardCharsets.UTF_8);
    Assert.assertTrue(text, text.contains("reference track: leg 0 does not run along it (end before start)"));
  }

  @Test
  public void aLegWithAStopOffTheTrackIsRoutedAsWithoutIt() {
    int[][] pair = TWO_WAY_PAIRS[0];
    List<OsmNode> line = backwards(route(pair, null));
    // the start of another pair, away from this one's old route
    int[][] offStart = {TWO_WAY_PAIRS[2][1], pair[0]};

    Assert.assertEquals(nodes(route(offStart, null)), nodes(route(offStart, line)));
  }
}
