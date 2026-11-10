package btools.router;

import java.io.File;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import btools.expressions.BExpressionContextWay;

/** Curvey: reading curvature tiles ({@link CurvatureTiles}), against files written from the spec. */
public class CurvatureTilesTest {
  @Rule
  public TemporaryFolder temp = new TemporaryFolder();

  private File profilesDir;
  private File segments;

  /** South-west corner of E10_N50 in microdegrees. */
  private static final int LON0 = 190000000;
  private static final int LAT0 = 140000000;
  private static final int CELL = 31250;

  @Before
  public void before() throws IOException {
    URL resource = getClass().getResource("/testtrack0.gpx");
    Assert.assertNotNull(resource);
    profilesDir = new File(new File(resource.getFile()).getParentFile(), "../../../../misc/profiles2");
    segments = temp.newFolder("segments");
  }

  /** shortest.brf with a cost that reads curvey_curve_class (twisty cheaper), in a folder with lookups.dat. */
  static File curvyProfile(File profilesDir, File target) throws IOException {
    target.mkdirs();
    Files.copy(new File(profilesDir, "lookups.dat").toPath(), new File(target, "lookups.dat").toPath(), StandardCopyOption.REPLACE_EXISTING);
    String shortest = new String(Files.readAllBytes(new File(profilesDir, "shortest.brf").toPath()), "UTF-8");
    String marker = "  switch or highway=proposed highway=abandoned        100000\n  1\n";
    Assert.assertTrue("shortest.brf changed, adapt the test profile", shortest.contains(marker));
    String curvy = shortest.replace(marker,
      "  switch or highway=proposed highway=abandoned        100000\n  switch curvey_curve_class=twisty 1 6\n");
    File profile = new File(target, "curvy-shortest.brf");
    Files.write(profile.toPath(), curvy.getBytes("UTF-8"));
    return profile;
  }

  private BExpressionContextWay context(File profile) {
    RoutingContext rc = new RoutingContext();
    rc.localFunction = profile.getAbsolutePath();
    ProfileCache.parseProfile(rc);
    return rc.expctxWay;
  }

  private CurvatureTiles tiles() throws IOException {
    CurvatureTiles tiles = CurvatureTiles.create(segments, context(curvyProfile(profilesDir, temp.newFolder())));
    Assert.assertNotNull(tiles);
    return tiles;
  }

  private void write(CurvatureTileTestWriter.Link... links) throws IOException {
    CurvatureTileTestWriter.write(CurvatureTileTestWriter.tileFile(segments, 10, 50), 10, 50, 32, Arrays.asList(links), 1, 1,
      Collections.singletonMap("osm_date", "2026-11-04"));
  }

  private static CurvatureTileTestWriter.Link link(int dLonA, int dLatA, int dLonB, int dLatB, int curveClass) {
    return new CurvatureTileTestWriter.Link(LON0 + dLonA, LAT0 + dLatA, LON0 + dLonB, LAT0 + dLatB, curveClass);
  }

  @Test
  public void aProfileThatNeverReadsTheKeyGetsNoHook() {
    Assert.assertNull(CurvatureTiles.create(segments, context(new File(profilesDir, "shortest.brf"))));
    Assert.assertNull(CurvatureTiles.create(segments, context(new File(profilesDir, "car-vario.brf"))));
  }

  @Test
  public void aLinkInsideOneCellIsFoundFromBothEnds() throws IOException {
    write(link(500, 500, 100, 900, 3));
    CurvatureTiles tiles = tiles();
    Assert.assertEquals(3, tiles.classOf(LON0 + 500, LAT0 + 500, LON0 + 100, LAT0 + 900));
    Assert.assertEquals(3, tiles.classOf(LON0 + 100, LAT0 + 900, LON0 + 500, LAT0 + 500));
  }

  @Test
  public void aLinkAcrossCellsIsFoundFromBothEnds() throws IOException {
    write(link(CELL - 100, 10, CELL + 400, 10, 2));
    CurvatureTiles tiles = tiles();
    Assert.assertEquals(2, tiles.classOf(LON0 + CELL - 100, LAT0 + 10, LON0 + CELL + 400, LAT0 + 10));
    Assert.assertEquals(2, tiles.classOf(LON0 + CELL + 400, LAT0 + 10, LON0 + CELL - 100, LAT0 + 10));
  }

  @Test
  public void aLinkAcrossTheTileEdgeIsFoundFromBothTiles() throws IOException {
    CurvatureTileTestWriter.writeAll(segments, Collections.singletonList(link(-200, 1000, 300, 1000, 4)));
    CurvatureTiles tiles = tiles();
    Assert.assertEquals(4, tiles.classOf(LON0 - 200, LAT0 + 1000, LON0 + 300, LAT0 + 1000));
    Assert.assertEquals(4, tiles.classOf(LON0 + 300, LAT0 + 1000, LON0 - 200, LAT0 + 1000));
  }

  @Test
  public void manyLinksRoundTrip() throws IOException {
    java.util.Random random = new java.util.Random(344);
    List<CurvatureTileTestWriter.Link> links = new java.util.ArrayList<>();
    java.util.Set<String> seen = new java.util.HashSet<>();
    while (links.size() < 3000) {
      int lon = random.nextInt(5000000);
      int lat = random.nextInt(5000000);
      CurvatureTileTestWriter.Link l = link(lon, lat, lon + random.nextInt(80000) - 40000, lat + random.nextInt(80000) - 40000, 1 + random.nextInt(4));
      String a = l.ilonA + "," + l.ilatA;
      String b = l.ilonB + "," + l.ilatB;
      if (a.equals(b) || !seen.add(a.compareTo(b) < 0 ? a + b : b + a)) continue;
      links.add(l);
    }
    CurvatureTileTestWriter.writeAll(segments, links);
    CurvatureTiles tiles = tiles();
    for (CurvatureTileTestWriter.Link l : links) {
      Assert.assertEquals(l.curveClass, tiles.classOf(l.ilonA, l.ilatA, l.ilonB, l.ilatB));
      Assert.assertEquals(l.curveClass, tiles.classOf(l.ilonB, l.ilatB, l.ilonA, l.ilatA));
    }
  }

  @Test
  public void anUnknownLinkIsStraight() throws IOException {
    write(link(500, 500, 600, 600, 3));
    Assert.assertEquals(0, tiles().classOf(LON0 + 500, LAT0 + 500, LON0 + 700, LAT0 + 600));
  }

  @Test
  public void aMissingFileIsNeutral() throws IOException {
    CurvatureTiles tiles = tiles();
    Assert.assertEquals(0, tiles.classOf(LON0 + 500, LAT0 + 500, LON0 + 600, LAT0 + 600));
    Assert.assertEquals("missing", tiles.problemAt(LON0 + 500, LAT0 + 500));
  }

  @Test
  public void anotherFormatOrClassTableVersionIsNeutral() throws IOException {
    File file = CurvatureTileTestWriter.tileFile(segments, 10, 50);
    List<CurvatureTileTestWriter.Link> links = Collections.singletonList(link(500, 500, 600, 600, 3));
    CurvatureTileTestWriter.write(file, 10, 50, 32, links, 2, 1, Collections.<String, String>emptyMap());
    CurvatureTiles tiles = tiles();
    Assert.assertEquals(0, tiles.classOf(LON0 + 500, LAT0 + 500, LON0 + 600, LAT0 + 600));
    Assert.assertEquals("format version 2", tiles.problemAt(LON0 + 500, LAT0 + 500));

    CurvatureTileTestWriter.write(file, 10, 50, 32, links, 1, 2, Collections.<String, String>emptyMap());
    tiles = tiles();
    Assert.assertEquals(0, tiles.classOf(LON0 + 500, LAT0 + 500, LON0 + 600, LAT0 + 600));
    Assert.assertEquals("class table version 2", tiles.problemAt(LON0 + 500, LAT0 + 500));
  }

  @Test
  public void aFileWithTheWrongCornerOrSizeIsNeutral() throws IOException {
    List<CurvatureTileTestWriter.Link> links = Collections.singletonList(link(500, 500, 600, 600, 3));
    File file = CurvatureTileTestWriter.tileFile(segments, 10, 50);
    CurvatureTileTestWriter.write(file, 15, 50, 32, links, 1, 1, Collections.<String, String>emptyMap());
    CurvatureTiles tiles = tiles();
    Assert.assertEquals(0, tiles.classOf(LON0 + 500, LAT0 + 500, LON0 + 600, LAT0 + 600));
    Assert.assertEquals("tile corner does not match the name", tiles.problemAt(LON0 + 500, LAT0 + 500));

    write(link(500, 500, 600, 600, 3));
    try (RandomAccessFile raf = new RandomAccessFile(file, "rw")) {
      raf.setLength(raf.length() - 1);
    }
    tiles = tiles();
    Assert.assertEquals(0, tiles.classOf(LON0 + 500, LAT0 + 500, LON0 + 600, LAT0 + 600));
    Assert.assertEquals("size does not match the cell table", tiles.problemAt(LON0 + 500, LAT0 + 500));
  }

  @Test
  public void aDamagedCellIsNeutralAndTheRestStillWorks() throws IOException {
    write(link(500, 500, 600, 600, 3), link(2 * CELL + 10, 10, 2 * CELL + 20, 10, 2));
    File file = CurvatureTileTestWriter.tileFile(segments, 10, 50);
    byte[] bytes = Files.readAllBytes(file.toPath());
    // Data starts after the 16-byte header, the metadata and the cell table. Cell 0 holds one record: count (1 byte),
    // offset 500 * 31250 + 500 (4-byte varint), dLon and dLat 100 (2 bytes each), then the class. Turn it into 9.
    int dataStart = 16 + (2 + "osm_date".length()) + (2 + "2026-11-04".length()) + 4 * (160 * 160 + 1);
    int classByte = dataStart + 1 + 4 + 2 + 2;
    Assert.assertEquals(3, bytes[classByte]);
    bytes[classByte] = 9;
    Files.write(file.toPath(), bytes);
    CurvatureTiles tiles = tiles();
    Assert.assertEquals(0, tiles.classOf(LON0 + 500, LAT0 + 500, LON0 + 600, LAT0 + 600));
    Assert.assertEquals(2, tiles.classOf(LON0 + 2 * CELL + 10, LAT0 + 10, LON0 + 2 * CELL + 20, LAT0 + 10));
  }

  @Test
  public void aTaggedLinkGetsAVariantWithTheClassAndKeepsItsOtherTags() throws IOException {
    write(link(500, 500, 600, 600, 4));
    File profile = curvyProfile(profilesDir, temp.newFolder());
    BExpressionContextWay ctx = context(profile);
    CurvatureTiles tiles = CurvatureTiles.create(segments, ctx);
    int[] lookupData = ctx.createNewLookupData();
    lookupData[ctx.getLookupKey("highway")] = 5;
    byte[] description = ctx.encode(lookupData);
    description = ctx.unify(description, 0, description.length);

    byte[] tagged = tiles.tagLink(LON0 + 500, LAT0 + 500, LON0 + 600, LAT0 + 600, description);
    Assert.assertNotSame(description, tagged);
    int[] decoded = ctx.createNewLookupData();
    ctx.decode(decoded, false, tagged);
    int key = ctx.getLookupKey(CurvatureTiles.LOOKUP_KEY);
    Assert.assertEquals("twisty is the fifth value after empty and unknown", 2 + 4, decoded[key]);
    Assert.assertEquals(5, decoded[ctx.getLookupKey("highway")]);
    Assert.assertSame("variants are cached", tagged, tiles.tagLink(LON0 + 600, LAT0 + 600, LON0 + 500, LAT0 + 500, description));
    Assert.assertSame("no record, no change", description, tiles.tagLink(LON0 + 1, LAT0 + 1, LON0 + 2, LAT0 + 2, description));
    Assert.assertEquals(3, tiles.lookups);
    Assert.assertEquals(2, tiles.hits);
    Assert.assertEquals(1, tiles.variantsBuilt);
  }
}
