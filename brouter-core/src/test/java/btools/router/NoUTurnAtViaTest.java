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
 * Hard no-U-turn rule at via points ({@code continueStraight}), on the Dreieich test segments.
 * Waypoints are {lat, lon}; every route is a loop through on-road vias in an order that makes the
 * unguarded router reverse at a via point.
 */
public class NoUTurnAtViaTest {
  private File workingDir;

  /** Loops with vias on roads; each reverses at least once without the rule. */
  private static final double[][][] REVERSING_LOOPS = {
    {{49.997500, 8.723700}, {49.999048, 8.724075}, {50.000528, 8.725140}, {50.001780, 8.719712}, {50.002362, 8.720945}, {50.000352, 8.715350}, {49.997500, 8.723700}},
    {{49.997500, 8.723700}, {49.997494, 8.723686}, {50.002409, 8.721041}, {49.996722, 8.724245}, {49.997500, 8.723700}},
    {{49.997500, 8.723700}, {50.001131, 8.716371}, {50.000219, 8.714883}, {50.001137, 8.720403}, {49.997500, 8.723700}},
    {{50.002500, 8.720900}, {49.997494, 8.723686}, {50.001846, 8.718417}, {50.002409, 8.721041}, {50.002500, 8.720900}},
  };

  @Before
  public void before() {
    URL resulturl = this.getClass().getResource("/testtrack0.gpx");
    Assert.assertNotNull("reference result not found: ", resulturl);
    workingDir = new File(resulturl.getFile()).getParentFile();
  }

  private RoutingEngine route(double[][] latLon, boolean continueStraight) {
    List<OsmNodeNamed> wplist = new ArrayList<>();
    for (int i = 0; i < latLon.length; i++) {
      OsmNodeNamed n = new OsmNodeNamed();
      n.name = i == 0 ? "from" : i == latLon.length - 1 ? "to" : "via" + i;
      n.ilat = 90000000 + (int) (latLon[i][0] * 1000000 + 0.5);
      n.ilon = 180000000 + (int) (latLon[i][1] * 1000000 + 0.5);
      wplist.add(n);
    }
    RoutingContext rctx = new RoutingContext();
    rctx.keyValues = new HashMap<>();
    if (continueStraight) rctx.keyValues.put("continueStraight", "1");
    rctx.localFunction = workingDir.getAbsolutePath() + "/../../../../misc/profiles2/trekking.brf";
    RoutingEngine re = new RoutingEngine(null, null,
      new File(workingDir, "/../../../../brouter-map-creator/build/resources/test/tmp/segments"), wplist, rctx);
    re.quite = true;
    re.doRun(0);
    return re;
  }

  /** Immediate reversals A,B,A in the track geometry. */
  private static int reversals(OsmTrack track) {
    int n = 0;
    for (int i = 2; i < track.nodes.size(); i++) {
      OsmPathElement a = track.nodes.get(i - 2);
      OsmPathElement c = track.nodes.get(i);
      if (a.getILon() == c.getILon() && a.getILat() == c.getILat()) n++;
    }
    return n;
  }

  @Test
  public void withoutTheRuleTheCorpusReverses() {
    for (double[][] loop : REVERSING_LOOPS) {
      RoutingEngine re = route(loop, false);
      Assert.assertNull(re.getErrorMessage(), re.getErrorMessage());
      Assert.assertTrue("premise: loop should reverse without the rule", reversals(re.getFoundTrack()) > 0);
    }
  }

  @Test
  public void continueStraightNeverReversesAtAViaPoint() {
    for (double[][] loop : REVERSING_LOOPS) {
      RoutingEngine re = route(loop, true);
      Assert.assertNull(re.getErrorMessage(), re.getErrorMessage());
      Assert.assertEquals(0, reversals(re.getFoundTrack()));
    }
  }

  @Test
  public void ruleIsOffByDefault() {
    // no profile variable: unchanged upstream behaviour (route found, reversal allowed)
    RoutingEngine re = route(REVERSING_LOOPS[0], false);
    Assert.assertNotNull(re.getFoundTrack());
  }
}
