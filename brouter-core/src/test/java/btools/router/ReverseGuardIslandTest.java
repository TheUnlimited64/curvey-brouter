package btools.router;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import java.io.File;
import java.net.URL;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * The no-U-turn guard must not disable BRouter's island recovery when it never rejected a step:
 * a start cut off by a ring of impassable nogos is handled the same way with and without an armed
 * guard (before the fix the armed guard skipped island handling and reported "without a U-turn").
 */
public class ReverseGuardIslandTest {
  private File workingDir;

  /** Inside the pocket that {@link #RING} cuts off (lat, lon). */
  private static final double[] ISLAND = {49.9998, 8.7225};
  private static final double[] TARGET = {49.997500, 8.723700};
  /** Radius (m) of the ring of nogo circles that cuts the start off. */
  private static final double RING_RADIUS_M = 70.;
  private static final int RING_CIRCLES = 24;
  private static final int RING_CIRCLE_RADIUS_M = 12;

  /** An annulus of small impassable circles around the start: everything inside is an island. */
  private static String ring() {
    int ilon = 180000000 + (int) (ISLAND[1] * 1000000 + 0.5);
    int ilat = 90000000 + (int) (ISLAND[0] * 1000000 + 0.5);
    StringBuilder sb = new StringBuilder();
    for (int k = 0; k < RING_CIRCLES; k++) {
      int[] p = btools.util.CheapRuler.destination(ilon, ilat, RING_RADIUS_M, 360. * k / RING_CIRCLES);
      if (sb.length() > 0) sb.append('|');
      sb.append((p[0] - 180000000) / 1e6).append(',').append((p[1] - 90000000) / 1e6).append(',').append(RING_CIRCLE_RADIUS_M);
    }
    return sb.toString();
  }

  @Before
  public void before() {
    URL resulturl = this.getClass().getResource("/testtrack0.gpx");
    Assert.assertNotNull("reference result not found: ", resulturl);
    workingDir = new File(resulturl.getFile()).getParentFile();
  }

  private RoutingEngine route(boolean guardArmed) {
    List<OsmNodeNamed> wplist = new ArrayList<>();
    double[][] pts = {ISLAND, TARGET};
    for (int i = 0; i < pts.length; i++) {
      OsmNodeNamed n = new OsmNodeNamed();
      n.name = i == 0 ? "from" : "to";
      n.ilat = 90000000 + (int) (pts[i][0] * 1000000 + 0.5);
      n.ilon = 180000000 + (int) (pts[i][1] * 1000000 + 0.5);
      wplist.add(n);
    }
    RoutingContext rctx = new RoutingContext();
    rctx.keyValues = new HashMap<>();
    rctx.keyValues.put("continueStraight", "1");
    if (guardArmed) {
      // a position that is no neighbour of the start: the guard is armed but can never reject a step
      rctx.arrivedFromLon = 180000000 + 8730000;
      rctx.arrivedFromLat = 90000000 + 50010000;
    }
    List<OsmNodeNamed> nogos = new RoutingParamCollector().readNogoList(ring());
    RoutingContext.prepareNogoPoints(nogos);
    rctx.nogopoints = nogos;
    rctx.localFunction = workingDir.getAbsolutePath() + "/../../../../misc/profiles2/trekking.brf";
    RoutingEngine re = new RoutingEngine(null, null,
      new File(workingDir, "/../../../../brouter-map-creator/build/resources/test/tmp/segments"), wplist, rctx);
    re.quite = true;
    re.doRun(0);
    return re;
  }

  @Test
  public void anArmedGuardThatNeverFiresKeepsIslandRecovery() {
    RoutingEngine free = route(false);
    RoutingEngine guarded = route(true);

    // the guard never rejected a step, so the outcome must not be blamed on it
    Assert.assertEquals(free.getErrorMessage(), guarded.getErrorMessage());
    Assert.assertEquals(free.getFoundTrack() == null, guarded.getFoundTrack() == null);
  }
}
