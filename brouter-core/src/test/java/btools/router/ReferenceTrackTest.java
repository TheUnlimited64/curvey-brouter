package btools.router;

import org.junit.Assert;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import btools.mapaccess.OsmNode;
import btools.util.CheapRuler;

/**
 * Geometry of {@link ReferenceTrack} (Curvey #458): which part of a route step follows the line, in its direction.
 * Positions are metres east/north of a point near Dreieich.
 */
public class ReferenceTrackTest {
  private static final int LON0 = 180_000_000 + 8_720_000;
  private static final int LAT0 = 90_000_000 + 50_000_000;
  private static final double[] K = CheapRuler.getLonLatToMeterScales(LAT0);

  /** Rounding of a position to 1e-5 degrees (a route stored as an encoded polyline), micro degrees. */
  private static final int STORED_ROUNDING = 10;

  private static int lon(double eastM) {
    return LON0 + (int) Math.round(eastM / K[0]);
  }

  private static int lat(double northM) {
    return LAT0 + (int) Math.round(northM / K[1]);
  }

  private static OsmNode at(double eastM, double northM) {
    return new OsmNode(lon(eastM), lat(northM));
  }

  /** The line through east/north pairs, in order. */
  private static ReferenceTrack line(double... eastNorth) {
    List<OsmNode> positions = new ArrayList<>();
    for (int i = 0; i + 1 < eastNorth.length; i += 2) positions.add(at(eastNorth[i], eastNorth[i + 1]));
    return ReferenceTrack.of(positions);
  }

  private static double along(ReferenceTrack line, double e1, double n1, double e2, double n2) {
    return line.alongM(lon(e1), lat(n1), lon(e2), lat(n2));
  }

  private static double length(double e1, double n1, double e2, double n2) {
    return CheapRuler.distance(lon(e1), lat(n1), lon(e2), lat(n2));
  }

  /** Rounding of geometry to whole micro degrees, about 0.1 m. */
  private static final double ROUNDING_M = 0.2;

  private final ReferenceTrack north = line(0, 0, 0, 500);

  @Test
  public void aStepOnTheLineInItsDirectionFollowsItAllTheWay() {
    Assert.assertEquals(length(0, 100, 0, 200), along(north, 0, 100, 0, 200), ROUNDING_M);
  }

  @Test
  public void aStepAgainstTheLineDoesNotFollowIt() {
    Assert.assertEquals(0., along(north, 0, 200, 0, 100), 0.);
  }

  @Test
  public void theOtherCarriagewayOfADividedRoadFollowsIt() {
    Assert.assertEquals(length(20, 100, 20, 200), along(north, 20, 100, 20, 200), ROUNDING_M);
    Assert.assertEquals(length(-20, 100, -20, 200), along(north, -20, 100, -20, 200), ROUNDING_M);
  }

  @Test
  public void aParallelRoadBeyondTheCorridorDoesNot() {
    double beyond = ReferenceTrack.ALONG_CORRIDOR_M + 5.;
    Assert.assertEquals(0., along(north, beyond, 100, beyond, 200), 0.);
  }

  @Test
  public void aRoadCrossingTheLineDoesNot() {
    Assert.assertEquals(0., along(north, -50, 100, 50, 100), 0.);
  }

  @Test
  public void aTurnIntoASideRoadDoesNot() {
    Assert.assertEquals(0., along(north, 0, 100, 25, 125), 0.); // 45 degrees off the line
  }

  @Test
  public void aStepDriftingAwayFollowsTheLineUpToTheCorridorsEdge() {
    // 20 degrees off the line: inside the corridor up to 30 m to the side
    double east = 200. * Math.tan(Math.toRadians(20.));
    double step = length(0, 100, east, 300);
    double expected = step * ReferenceTrack.ALONG_CORRIDOR_M / east;
    Assert.assertEquals(expected, along(north, 0, 100, east, 300), 1.);
  }

  @Test
  public void aStepPastTheEndFollowsTheLineOnlyToItsEnd() {
    Assert.assertEquals(length(0, 450, 0, 500), along(north, 0, 450, 0, 550), ROUNDING_M);
    Assert.assertEquals(length(0, 0, 0, 50), along(north, 0, -50, 0, 50), ROUNDING_M);
  }

  @Test
  public void aLongStepFollowsALineOfShortSteps() {
    ReferenceTrack line = line(0, 0, 0, 100, 0, 200, 0, 300);
    Assert.assertEquals(length(0, 50, 0, 250), along(line, 0, 50, 0, 250), ROUNDING_M);
  }

  @Test
  public void aLineRiddenTwiceTheSameWayCountsOnce() {
    ReferenceTrack twice = line(0, 0, 0, 300, 100, 300, 100, 0, 0, 0, 0, 300);
    Assert.assertEquals(length(0, 100, 0, 200), along(twice, 0, 100, 0, 200), ROUNDING_M);
  }

  @Test
  public void anOutAndBackLineIsFollowedBothWays() {
    ReferenceTrack spur = line(0, 0, 0, 300, 0, 0);
    Assert.assertEquals(length(0, 100, 0, 200), along(spur, 0, 100, 0, 200), ROUNDING_M);
    Assert.assertEquals(length(0, 200, 0, 100), along(spur, 0, 200, 0, 100), ROUNDING_M);
  }

  @Test
  public void aStoredRouteRoundedTo1e5DegreesStillLiesOnTheRoad() {
    // a bend of radius 150 m in 5 degree steps; the line is the same road rounded as an encoded polyline stores it
    List<OsmNode> road = new ArrayList<>();
    List<OsmNode> stored = new ArrayList<>();
    for (int deg = 0; deg <= 90; deg += 5) {
      double a = Math.toRadians(deg);
      OsmNode p = at(150. * (1. - Math.cos(a)), 150. * Math.sin(a));
      road.add(p);
      stored.add(new OsmNode(Math.round(p.ilon / (float) STORED_ROUNDING) * STORED_ROUNDING,
        Math.round(p.ilat / (float) STORED_ROUNDING) * STORED_ROUNDING));
    }
    ReferenceTrack line = ReferenceTrack.of(stored);
    for (int i = 1; i < road.size(); i++) {
      OsmNode a = road.get(i - 1), b = road.get(i);
      double step = CheapRuler.distance(a.ilon, a.ilat, b.ilon, b.ilat);
      Assert.assertEquals("step " + i, step, line.alongM(a.ilon, a.ilat, b.ilon, b.ilat), ROUNDING_M);
      Assert.assertEquals("step " + i + " backwards", 0., line.alongM(b.ilon, b.ilat, a.ilon, a.ilat), 0.);
    }
  }

  @Test
  public void aLongLineAnswersAsEveryStepAloneWould() {
    // a random road of 3000 steps with sharp turns, and random route steps near it
    Random random = new Random(458);
    List<OsmNode> positions = new ArrayList<>();
    double e = 0, n = 0, heading = 0;
    positions.add(at(e, n));
    for (int i = 0; i < 3000; i++) {
      heading += (random.nextDouble() - 0.5) * (i % 50 == 0 ? Math.PI : 0.6);
      double step = 5. + random.nextDouble() * 60.;
      e += step * Math.sin(heading);
      n += step * Math.cos(heading);
      positions.add(at(e, n));
    }
    ReferenceTrack line = ReferenceTrack.of(positions);
    List<ReferenceTrack> steps = new ArrayList<>();
    for (int i = 1; i < positions.size(); i++) steps.add(ReferenceTrack.of(positions.subList(i - 1, i + 1)));
    for (int q = 0; q < 2000; q++) {
      OsmNode p = positions.get(random.nextInt(positions.size()));
      int fromLon = p.ilon + random.nextInt(800) - 400, fromLat = p.ilat + random.nextInt(800) - 400;
      int toLon = fromLon + random.nextInt(2000) - 1000, toLat = fromLat + random.nextInt(2000) - 1000;
      double stepLen = CheapRuler.distance(fromLon, fromLat, toLon, toLat);
      double scan = 0.;
      for (ReferenceTrack s : steps) scan += s.alongM(fromLon, fromLat, toLon, toLat);
      Assert.assertEquals("query " + q, Math.min(scan, stepLen), line.alongM(fromLon, fromLat, toLon, toLat), 1e-6);
    }
  }

  @Test
  public void aLongStepAnswersAsItsShortPiecesWould() {
    // a straight road running north-east for about 20 km, and route steps over many grid cells along and across it;
    // the reference is the same step in pieces shorter than a cell, which never need more than the cells of their
    // box (straight, so no two line steps project onto the same part of a route step, which a whole step counts twice)
    Random random = new Random(4581);
    List<OsmNode> positions = new ArrayList<>();
    for (int i = 0; i <= 400; i++) positions.add(at(i * 35., i * 35.));
    ReferenceTrack line = ReferenceTrack.of(positions);
    double totalAlong = 0.;
    for (int q = 0; q < 300; q++) {
      OsmNode p = positions.get(random.nextInt(positions.size()));
      int fromLon = p.ilon + random.nextInt(200) - 100, fromLat = p.ilat + random.nextInt(200) - 100;
      int reach = 4_000 + random.nextInt(16_000);
      boolean alongTheRoad = random.nextBoolean();
      int toLon = fromLon + reach, toLat = fromLat + (alongTheRoad ? (int) (reach * K[0] / K[1]) : -reach);
      int pieces = 1 + reach / 500;
      double inPieces = 0.;
      for (int k = 0; k < pieces; k++) {
        inPieces += line.alongM(
          fromLon + (int) ((long) (toLon - fromLon) * k / pieces), fromLat + (int) ((long) (toLat - fromLat) * k / pieces),
          fromLon + (int) ((long) (toLon - fromLon) * (k + 1) / pieces), fromLat + (int) ((long) (toLat - fromLat) * (k + 1) / pieces));
      }
      // micro degree rounding bends the straight road a little: a few decimetres per kilometre
      Assert.assertEquals("query " + q, inPieces, line.alongM(fromLon, fromLat, toLon, toLat), Math.max(1., inPieces * 0.002));
      totalAlong += inPieces;
    }
    Assert.assertTrue("premise: the steps along the road follow it", totalAlong > 10_000.);
  }

  @Test
  public void passesNearAPositionAreFoundOnTheLine() {
    ReferenceTrack line = line(0, 0, 0, 500, 500, 500);
    double[] start = line.passesNear(lon(10), lat(0));
    Assert.assertEquals(0., start[0], ROUNDING_M);
    Assert.assertEquals(0., start[1], ROUNDING_M);
    double[] middle = line.passesNear(lon(5), lat(250));
    Assert.assertEquals(250., middle[0], 1.);
    Assert.assertEquals(250., middle[1], 1.);
    Assert.assertNull(line.passesNear(lon(100), lat(250)));
  }

  @Test
  public void aLoopPassesItsStartAtBothEnds() {
    ReferenceTrack loop = line(0, 0, 0, 500, 500, 500, 500, 0, 0, 0);
    double[] start = loop.passesNear(lon(0), lat(0));
    Assert.assertEquals(0., start[0], 1.);
    Assert.assertEquals(loop.lengthM(), start[1], 1.);
  }

  @Test
  public void fewerThanTwoPositionsAreNoLine() {
    Assert.assertNull(ReferenceTrack.of(null));
    Assert.assertNull(ReferenceTrack.of(new ArrayList<>()));
    List<OsmNode> same = new ArrayList<>();
    same.add(at(0, 0));
    same.add(at(0, 0));
    Assert.assertNull(ReferenceTrack.of(same));
  }

  @Test
  public void repeatedPositionsAreDropped() {
    ReferenceTrack line = line(0, 0, 0, 0, 0, 100, 0, 100, 0, 200);
    Assert.assertEquals(3, line.size());
    Assert.assertEquals(200., line.lengthM(), 1.);
  }

  @Test
  public void aStepOfNoLengthFollowsNothing() {
    Assert.assertEquals(0., along(north, 0, 100, 0, 100), 0.);
  }
}
