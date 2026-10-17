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

  /**
   * Profile of the routes: trekking (cost = costfactor x distance) by default. Car profiles use the kinematic
   * model, where the cost is time and costfactor only an add-on penalty, usually 0.
   */
  private String profile = "trekking";

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
    return route(latLon, continueStraight, null);
  }

  private RoutingEngine route(double[][] latLon, boolean continueStraight, int[] arrivedFromLonLat) {
    return route(latLon, continueStraight, arrivedFromLonLat, null);
  }

  private RoutingEngine route(double[][] latLon, boolean continueStraight, int[] arrivedFromLonLat, Double reversalCostFactor) {
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
    if (reversalCostFactor != null) rctx.keyValues.put("reversalCostFactor", String.valueOf(reversalCostFactor));
    if (arrivedFromLonLat != null) {
      // one or more lon,lat pairs, oldest first
      rctx.arrivedFrom = new ArrayList<>();
      for (int k = 0; k + 1 < arrivedFromLonLat.length; k += 2) {
        rctx.arrivedFrom.add(new btools.mapaccess.OsmNode(arrivedFromLonLat[k], arrivedFromLonLat[k + 1]));
      }
    }
    rctx.localFunction = workingDir.getAbsolutePath() + "/../../../../misc/profiles2/" + profile + ".brf";
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

  /**
   * Metres of {@code track} that run backwards along the {@link RoutingContext#DRIVEN_TRACE_MAX_LENGTH_M} before them,
   * measured with the engine's own R2 test: a later leg coming back along an earlier one.
   */
  private static double backAlongItself(OsmTrack track) {
    List<btools.mapaccess.OsmNode> before = new ArrayList<>();
    double back = 0;
    for (int k = 0; k + 1 < track.nodes.size(); k++) {
      OsmPathElement a = track.nodes.get(k);
      OsmPathElement b = track.nodes.get(k + 1);
      before.add(new btools.mapaccess.OsmNode(a.getILon(), a.getILat()));
      DrivenTrace t = DrivenTrace.tail(before, RoutingContext.DRIVEN_TRACE_MAX_LENGTH_M);
      if (t != null) back += t.backwardOverlapM(a.getILon(), a.getILat(), b.getILon(), b.getILat());
    }
    return back;
  }

  /**
   * R1 stops the reversal at a via; R2 also makes the next leg avoid coming back along the previous one later
   * (after a loop through a side street). Never more of it than without R2, and less on some loop.
   */
  @Test
  public void goingBackAlongThePreviousLegCostsExtra() {
    assertR2ReducesGoingBack();
  }

  @Test
  public void goingBackAlongThePreviousLegCostsExtraWithTheKinematicCarProfile() {
    profile = "car-vario";
    assertR2ReducesGoingBack();
  }

  private void assertR2ReducesGoingBack() {
    int reduced = 0;
    for (double[][] loop : REVERSING_LOOPS) {
      RoutingEngine withoutR2 = route(loop, true, null, 1.);
      RoutingEngine withR2 = route(loop, true, null, null);
      if (withoutR2.getErrorMessage() != null) continue; // a via on a dead end for this profile: fails by R1 alone
      Assert.assertNull("R2 never makes a route fail: " + withR2.getErrorMessage(), withR2.getErrorMessage());
      Assert.assertEquals("R2 never brings a reversal back", 0, reversals(withR2.getFoundTrack()));

      double before = backAlongItself(withoutR2.getFoundTrack());
      double after = backAlongItself(withR2.getFoundTrack());
      Assert.assertTrue("loop from " + loop[0][0] + ": " + after + " m back with R2, " + before + " m without", after <= before + 1.);
      if (after < before - 10.) reduced++;
    }
    Assert.assertTrue("premise: some loop comes back along a previous leg without R2", reduced > 0);
  }

  @Test
  public void ruleIsOffByDefault() {
    // no profile variable: unchanged upstream behaviour (route found, reversal allowed)
    RoutingEngine re = route(REVERSING_LOOPS[0], false);
    Assert.assertNotNull(re.getFoundTrack());
  }

  @Test
  public void arrivedFromGuardsTheFirstLeg() {
    double[][] pair = {{49.997500, 8.723700}, {50.002409, 8.721041}};
    OsmTrack free = route(pair, true).getFoundTrack();
    OsmPathElement start = free.nodes.get(0);
    OsmPathElement firstStep = null;
    for (OsmPathElement e : free.nodes) {
      if (btools.util.CheapRuler.distance(e.getILon(), e.getILat(), start.getILon(), start.getILat()) > 3) {
        firstStep = e;
        break;
      }
    }
    Assert.assertNotNull(firstStep);

    // pretend the rider just arrived from where this route would go first
    RoutingEngine guarded = route(pair, true, new int[]{firstStep.getILon(), firstStep.getILat()});
    if (guarded.getErrorMessage() == null) {
      for (OsmPathElement e : guarded.getFoundTrack().nodes) {
        Assert.assertFalse("first step must not retrace the arrival edge",
          e.getILon() == firstStep.getILon() && e.getILat() == firstStep.getILat()
            && guarded.getFoundTrack().nodes.indexOf(e) < 3);
      }
    }
    // without the continueStraight rule the hint is ignored
    RoutingEngine ignored = route(pair, false, new int[]{firstStep.getILon(), firstStep.getILat()});
    Assert.assertNull(ignored.getErrorMessage());
  }

  /** The free track's first node more than 2 m from its start: the engine's own first step. */
  private static OsmPathElement engineFirstStep(OsmTrack t) {
    OsmPathElement s = t.nodes.get(0);
    for (OsmPathElement e : t.nodes) {
      if (btools.util.CheapRuler.distance(e.getILon(), e.getILat(), s.getILon(), s.getILat()) > 2) return e;
    }
    return null;
  }

  @Test
  public void arrivedFromListPicksTheLastPositionClearOfTheStart() {
    double[][] pair = {{49.997500, 8.723700}, {50.002409, 8.721041}};
    OsmTrack free = route(pair, true).getFoundTrack();
    OsmPathElement start = free.nodes.get(0);
    OsmPathElement step = engineFirstStep(free);
    OsmPathElement older = free.nodes.get(free.nodes.indexOf(step) + 1);
    // 1 m from the start: inside the engine's tolerance, so it must be skipped, not used
    int[] near = btools.util.CheapRuler.destination(start.getILon(), start.getILat(), 1., 90.);

    RoutingEngine single = route(pair, true, new int[]{step.getILon(), step.getILat()});
    RoutingEngine list = route(pair, true, new int[]{
      older.getILon(), older.getILat(), step.getILon(), step.getILat(), near[0], near[1]});

    Assert.assertEquals(single.getErrorMessage(), list.getErrorMessage());
    if (single.getFoundTrack() != null) {
      Assert.assertEquals(single.getFoundTrack().nodes.size(), list.getFoundTrack().nodes.size());
      Assert.assertEquals(single.getFoundTrack().distance, list.getFoundTrack().distance);
    }
  }

  @Test
  public void arrivedFromListWithEveryPositionAtTheStartArmsNoGuard() {
    double[][] pair = {{49.997500, 8.723700}, {50.002409, 8.721041}};
    OsmTrack free = route(pair, true).getFoundTrack();
    OsmPathElement start = free.nodes.get(0);
    int[] near = btools.util.CheapRuler.destination(start.getILon(), start.getILat(), 1., 90.);

    RoutingEngine guarded = route(pair, true, new int[]{start.getILon(), start.getILat(), near[0], near[1]});

    Assert.assertNull(guarded.getErrorMessage());
    Assert.assertEquals(free.distance, guarded.getFoundTrack().distance);
  }
}
