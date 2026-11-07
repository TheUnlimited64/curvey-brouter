package btools.router;

import org.junit.Assert;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import btools.mapaccess.OsmNode;

/**
 * Curvey #348: a long trace (a whole roundtrip so far) indexes its steps in a grid. The grid must find
 * exactly the overlaps the plain scan finds; it only skips steps that are far away.
 */
public class DrivenTraceGridTest {

  /** Zig-zag path (micro degrees) with {@code steps} steps of about 110 m, spread over several grid cells. */
  private static List<OsmNode> path(int steps) {
    List<OsmNode> out = new ArrayList<>();
    int lon = 188_000_000, lat = 140_000_000;
    for (int i = 0; i <= steps; i++) {
      out.add(new OsmNode(lon, lat));
      lon += 1_000;
      lat += (i / 10) % 2 == 0 ? 600 : -600;
    }
    return out;
  }

  private static double scanOverlap(List<OsmNode> nodes, int fromLon, int fromLat, int toLon, int toLat) {
    double total = 0.;
    for (int i = 1; i < nodes.size(); i++) {
      // a two-point trace never uses the grid: the plain scan, step by step
      DrivenTrace one = DrivenTrace.tail(nodes.subList(i - 1, i + 1), 1e9);
      if (one != null) total += one.backwardOverlapM(fromLon, fromLat, toLon, toLat);
    }
    return total;
  }

  @Test
  public void gridFindsTheSameOverlapsAsTheScan() {
    List<OsmNode> nodes = path(500);
    DrivenTrace grid = DrivenTrace.tail(nodes, 1e9);
    Assert.assertEquals(nodes.size(), grid.size());
    for (int i = 1; i < nodes.size(); i += 7) {
      OsmNode a = nodes.get(i - 1), b = nodes.get(i);
      // backwards along step i, forwards along it, and a step beside the path
      double back = grid.backwardOverlapM(b.getILon(), b.getILat(), a.getILon(), a.getILat());
      Assert.assertEquals(scanOverlap(nodes, b.getILon(), b.getILat(), a.getILon(), a.getILat()), back, 1e-6);
      Assert.assertTrue("backwards along step " + i, back > 50.);
      Assert.assertEquals(0., grid.backwardOverlapM(a.getILon(), a.getILat(), b.getILon(), b.getILat()), 0.);
      Assert.assertEquals(0., grid.backwardOverlapM(b.getILon(), b.getILat() + 500, a.getILon(), a.getILat() + 500), 0.);
    }
  }

  @Test
  public void aStepAcrossGridCellsCountsEachDrivenStepOnce() {
    List<OsmNode> nodes = path(200);
    DrivenTrace grid = DrivenTrace.tail(nodes, 1e9);
    OsmNode from = nodes.get(25), to = nodes.get(21); // backwards over four driven steps, straight part of the zig-zag
    Assert.assertEquals(scanOverlap(nodes, from.getILon(), from.getILat(), to.getILon(), to.getILat()),
      grid.backwardOverlapM(from.getILon(), from.getILat(), to.getILon(), to.getILat()), 1e-6);
  }
}
