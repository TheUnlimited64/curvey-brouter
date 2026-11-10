package btools.router;

import java.io.File;
import java.io.IOException;
import java.net.URL;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import btools.codec.DataBuffers;
import btools.codec.WaypointMatcher;
import btools.expressions.BExpressionContextWay;
import btools.mapaccess.OsmFile;
import btools.mapaccess.PhysicalFile;

/**
 * Curvey: routing with curvature tiles on the Dreieich test segments (they span E5_N45 and E5_N50, so links cross a
 * tile edge). Parity: without a profile that reads {@code curvey_curve_class}, or without records, routes are the same
 * as without tiles.
 */
public class CurvatureTilesRoutingTest {
  @Rule
  public TemporaryFolder temp = new TemporaryFolder();

  private File workingDir;
  private File profilesDir;
  private File testSegments;

  private static final double[][] FROM_TO = {{49.997500, 8.723700}, {50.002409, 8.721041}};

  @Before
  public void before() {
    URL resource = getClass().getResource("/testtrack0.gpx");
    Assert.assertNotNull(resource);
    workingDir = new File(resource.getFile()).getParentFile();
    profilesDir = new File(workingDir, "../../../../misc/profiles2");
    testSegments = new File(workingDir, "../../../../brouter-map-creator/build/resources/test/tmp/segments");
  }

  /** A copy of the test segments, so curvature tiles can be put next to them. */
  private File segmentsCopy() throws IOException {
    File dir = temp.newFolder();
    for (File f : testSegments.listFiles()) {
      Files.copy(f.toPath(), new File(dir, f.getName()).toPath());
    }
    return dir;
  }

  private RoutingEngine route(File segments, File profile) {
    List<OsmNodeNamed> wplist = new ArrayList<>();
    for (int i = 0; i < FROM_TO.length; i++) {
      OsmNodeNamed n = new OsmNodeNamed();
      n.name = i == 0 ? "from" : "to";
      n.ilat = 90000000 + (int) (FROM_TO[i][0] * 1000000 + 0.5);
      n.ilon = 180000000 + (int) (FROM_TO[i][1] * 1000000 + 0.5);
      wplist.add(n);
    }
    RoutingContext rctx = new RoutingContext();
    rctx.localFunction = profile.getAbsolutePath();
    rctx.keyValues = new HashMap<>();
    RoutingEngine re = new RoutingEngine(null, null, segments, wplist, rctx);
    re.quite = true;
    re.doRun(0);
    Assert.assertNull(re.getErrorMessage(), re.getErrorMessage());
    return re;
  }

  private static List<Long> points(OsmTrack track) {
    List<Long> points = new ArrayList<>();
    for (OsmPathElement e : track.nodes) points.add(CurvatureTileTestWriter.point(e.getILon(), e.getILat()));
    return points;
  }

  /** Every link of the test segments that the profile can use, as {ilonA, ilatA, ilonB, ilatB}. */
  private List<int[]> allLinks(File segments, File profile) throws Exception {
    RoutingContext rc = new RoutingContext();
    rc.localFunction = profile.getAbsolutePath();
    ProfileCache.parseProfile(rc);
    BExpressionContextWay ctx = rc.expctxWay;
    List<int[]> links = new ArrayList<>();
    WaypointMatcher collector = new WaypointMatcher() {
      @Override
      public boolean start(int ilonStart, int ilatStart, int ilonTarget, int ilatTarget, boolean useAsStartWay) {
        links.add(new int[] {ilonStart, ilatStart, ilonTarget, ilatTarget});
        return false;
      }

      @Override
      public void transferNode(int ilon, int ilat) {
      }

      @Override
      public void end() {
      }

      @Override
      public boolean hasMatch(int lon, int lat) {
        return false;
      }
    };
    for (String name : new String[] {"E5_N45", "E5_N50"}) {
      DataBuffers buffers = new DataBuffers();
      PhysicalFile physical = new PhysicalFile(new File(segments, name + ".rd5"), buffers, ctx.meta.lookupVersion, ctx.meta.lookupMinorVersion);
      int lon0 = 185; // E5: (5 + 180) degrees
      int lat0 = name.endsWith("N45") ? 135 : 140;
      for (int dLon = 0; dLon < 5; dLon++) {
        for (int dLat = 0; dLat < 5; dLat++) {
          OsmFile osm = new OsmFile(physical, lon0 + dLon, lat0 + dLat, buffers);
          if (!osm.hasData()) continue;
          for (int i = 0; i < physical.divisor; i++) {
            for (int j = 0; j < physical.divisor; j++) {
              osm.createMicroCache((lon0 + dLon) * physical.divisor + i, (lat0 + dLat) * physical.divisor + j, buffers, ctx, collector, true, null);
            }
          }
        }
      }
      physical.close();
    }
    return links;
  }

  @Test
  public void aProfileThatDoesNotReadTheKeyRoutesAsWithoutTiles() throws Exception {
    File shortest = new File(profilesDir, "shortest.brf");
    File segments = segmentsCopy();
    List<Long> without = points(route(segments, shortest).getFoundTrack());
    List<CurvatureTileTestWriter.Link> twisty = new ArrayList<>();
    for (int[] l : allLinks(segments, shortest)) twisty.add(new CurvatureTileTestWriter.Link(l[0], l[1], l[2], l[3], 4));
    CurvatureTileTestWriter.writeAll(segments, twisty);
    RoutingEngine with = route(segments, shortest);
    Assert.assertNull(with.getCurvatureTiles());
    Assert.assertEquals(without, points(with.getFoundTrack()));
  }

  @Test
  public void withoutRecordsTheRouteIsUnchanged() throws Exception {
    File curvy = CurvatureTilesTest.curvyProfile(profilesDir, temp.newFolder());
    File segments = segmentsCopy();
    List<Long> without = points(route(segments, curvy).getFoundTrack());
    CurvatureTileTestWriter.writeAll(segments, new ArrayList<CurvatureTileTestWriter.Link>() {{
      add(new CurvatureTileTestWriter.Link(185000000, 140000000, 185000100, 140000000, 4)); // far from the route
    }});
    RoutingEngine with = route(segments, curvy);
    Assert.assertNotNull(with.getCurvatureTiles());
    Assert.assertEquals(0, with.getCurvatureTiles().hits);
    Assert.assertEquals(without, points(with.getFoundTrack()));
  }

  @Test
  public void cheaperCurvyLinksPullTheRouteOntoThem() throws Exception {
    File curvy = CurvatureTilesTest.curvyProfile(profilesDir, temp.newFolder());
    File segments = segmentsCopy();
    OsmTrack before = route(segments, curvy).getFoundTrack();
    Set<Long> onRoute = new HashSet<>(points(before));

    // Every link off today's route is twisty (cost 1 instead of 6 per metre): the route must move off it.
    List<CurvatureTileTestWriter.Link> offRoute = new ArrayList<>();
    for (int[] l : allLinks(segments, curvy)) {
      boolean on = onRoute.contains(CurvatureTileTestWriter.point(l[0], l[1])) && onRoute.contains(CurvatureTileTestWriter.point(l[2], l[3]));
      if (!on) offRoute.add(new CurvatureTileTestWriter.Link(l[0], l[1], l[2], l[3], 4));
    }
    CurvatureTileTestWriter.writeAll(segments, offRoute);
    Assert.assertTrue("both test tiles get a curvature tile", new File(segments, "E5_N45.curv").isFile() && new File(segments, "E5_N50.curv").isFile());

    RoutingEngine re = route(segments, curvy);
    OsmTrack after = re.getFoundTrack();
    Assert.assertNotEquals(points(before), points(after));
    Assert.assertTrue(re.getCurvatureTiles().hits > 0);
    Assert.assertTrue("the new route is cheaper under the curvature cost", after.cost < before.cost);
  }
}
