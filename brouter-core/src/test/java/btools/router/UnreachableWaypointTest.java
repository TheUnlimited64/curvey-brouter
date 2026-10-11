package btools.router;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import java.io.File;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;

/**
 * Typed error for an unreachable waypoint (#201). A hard nogo ring cuts the road network so that the
 * last waypoint ends up on a disconnected island of the Dreieich test segments.
 */
public class UnreachableWaypointTest {
  private File workingDir;

  @Before
  public void before() {
    URL resulturl = this.getClass().getResource("/testtrack0.gpx");
    Assert.assertNotNull("reference result not found: ", resulturl);
    workingDir = new File(resulturl.getFile()).getParentFile();
  }

  private static OsmNodeNamed waypoint(String name, double lat, double lon) {
    OsmNodeNamed n = new OsmNodeNamed();
    n.name = name;
    n.ilat = 90000000 + (int) (lat * 1000000 + 0.5);
    n.ilon = 180000000 + (int) (lon * 1000000 + 0.5);
    return n;
  }

  private RoutingEngine route(List<OsmNodeNamed> waypoints, boolean cutOffTarget) {
    RoutingContext rctx = new RoutingContext();
    if (cutOffTarget) {
      OsmNodeNamed ring = waypoint("ring", 49.9962, 8.7244);
      ring.isNogo = true;
      ring.radius = 90;
      ring.nogoWeight = Double.NaN;
      rctx.nogopoints = new ArrayList<>();
      rctx.nogopoints.add(ring);
    }
    rctx.localFunction = workingDir.getAbsolutePath() + "/../../../../misc/profiles2/trekking.brf";
    RoutingEngine re = new RoutingEngine(null, null,
      new File(workingDir, "/../../../../brouter-map-creator/build/resources/test/tmp/segments"), waypoints, rctx);
    re.quite = true;
    re.doRun(0);
    return re;
  }

  @Test
  public void unreachableTargetReportsItsWaypointIndex() {
    List<OsmNodeNamed> wps = new ArrayList<>();
    wps.add(waypoint("from", 49.997500, 8.723700));
    wps.add(waypoint("to", 49.995422, 8.725602));

    RoutingEngine re = route(wps, true);

    Assert.assertEquals(1, re.getUnreachableWaypointIndex());
    Assert.assertEquals("target island detected for section 0", re.getErrorMessage());
  }

  @Test
  public void unreachableViaReportsItsOwnIndexNotTheLastOne() {
    List<OsmNodeNamed> wps = new ArrayList<>();
    wps.add(waypoint("from", 49.997500, 8.723700));
    wps.add(waypoint("via1", 49.995422, 8.725602));
    wps.add(waypoint("to", 49.997494, 8.723686));

    RoutingEngine re = route(wps, true);

    Assert.assertEquals(1, re.getUnreachableWaypointIndex());
  }

  @Test
  public void reachableRouteReportsNoUnreachableWaypoint() {
    List<OsmNodeNamed> wps = new ArrayList<>();
    wps.add(waypoint("from", 49.997500, 8.723700));
    wps.add(waypoint("to", 49.995422, 8.725602));

    RoutingEngine re = route(wps, false);

    Assert.assertNull(re.getErrorMessage());
    Assert.assertEquals(-1, re.getUnreachableWaypointIndex());
  }
}
