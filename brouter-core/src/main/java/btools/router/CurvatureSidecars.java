package btools.router;

import java.io.File;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.Map;

import btools.codec.LinkTagHook;
import btools.expressions.BExpressionContextWay;

/**
 * EXPERIMENT (#341, Curvey fork, not for merge): optional curvature side-car files next to the routing tiles
 * ({@code E10_N50.curv} next to {@code E10_N50.rd5}). While a cell is woven, each link found in the side-car gets the
 * pseudo-tag {@code curvey_curve_class} (lookups.dat values straight..twisty = class 0..4) added to its way
 * description, so the profile prices it with normal syntax. No file, no entry, or a profile that never reads the
 * key: the description is left alone, so routing is unchanged.
 *
 * Prototype format v0 (big-endian): "CURV", u16 version 0, i16 tile lon, i16 tile lat, u16 divisor (cells per
 * degree), int[cells + 1] entry offsets (cells = (5 * divisor)^2, row-major by lat then lon), then 13-byte entries
 * sorted by cell of their node A: u16 dlonA, u16 dlatA (from the cell origin), i32 dlonB, i32 dlatB (B - A), u8 class.
 * Every link is stored under both of its end nodes' cells, A and B swapped.
 */
public final class CurvatureSidecars implements LinkTagHook {
  private static final int ENTRY_BYTES = 13;
  private static final int TILE_DEG = 5;
  private static final int VALUE_OFFSET = 2; // 0 = empty, 1 = unknown, then lookups.dat order

  private final File segmentDir;
  private final BExpressionContextWay ctx;
  private final int keyIdx;
  private final Map<Integer, Tile> tiles = new HashMap<>();
  private final IdentityHashMap<byte[], byte[][]> variants = new IdentityHashMap<>();
  public long lookups, hits, variantsBuilt, cellsLoaded, bytesLoaded;

  private static final class Tile {
    RandomAccessFile file;
    int lon0Cell, lat0Cell, cellsPerSide, divisor;
    int[] offsets;
    final Map<Integer, byte[]> cells = new HashMap<>();
  }

  private CurvatureSidecars(File segmentDir, BExpressionContextWay ctx, int keyIdx) {
    this.segmentDir = segmentDir;
    this.ctx = ctx;
    this.keyIdx = keyIdx;
  }

  /** A hook for {@code ctx}'s profile, or null when the profile never reads {@code curvey_curve_class}. */
  public static CurvatureSidecars create(File segmentDir, BExpressionContextWay ctx) {
    if (ctx == null || segmentDir == null) return null;
    int idx = ctx.getLookupKey("curvey_curve_class");
    if (idx < 0 || !ctx.isLookupIdxUsed(idx)) return null;
    return new CurvatureSidecars(segmentDir, ctx, idx);
  }

  @Override
  public byte[] tagLink(int ilonA, int ilatA, int ilonB, int ilatB, byte[] description) {
    lookups++;
    int cls = lookup(ilonA, ilatA, ilonB, ilatB);
    if (cls < 0) return description;
    hits++;
    byte[][] v = variants.get(description);
    if (v == null) {
      v = new byte[5][];
      variants.put(description, v);
    }
    byte[] r = v[cls];
    if (r == null) {
      int[] ld = ctx.createNewLookupData();
      ctx.decode(ld, false, description);
      ld[keyIdx] = VALUE_OFFSET + cls;
      byte[] enc = ctx.encode(ld);
      r = ctx.unify(enc, 0, enc.length);
      v[cls] = r;
      variantsBuilt++;
    }
    return r;
  }

  int lookup(int ilonA, int ilatA, int ilonB, int ilatB) {
    Tile t = tile(ilonA, ilatA);
    if (t == null) return -1;
    int cellSize = 1000000 / t.divisor;
    int cx = ilonA / cellSize - t.lon0Cell;
    int cy = ilatA / cellSize - t.lat0Cell;
    if (cx < 0 || cy < 0 || cx >= t.cellsPerSide || cy >= t.cellsPerSide) return -1;
    int cell = cy * t.cellsPerSide + cx;
    byte[] data = cellData(t, cell);
    if (data == null) return -1;
    int dlonA = ilonA - (t.lon0Cell + cx) * cellSize;
    int dlatA = ilatA - (t.lat0Cell + cy) * cellSize;
    int dlonB = ilonB - ilonA;
    int dlatB = ilatB - ilatA;
    for (int p = 0; p + ENTRY_BYTES <= data.length; p += ENTRY_BYTES) {
      int ea = ((data[p] & 0xff) << 8) | (data[p + 1] & 0xff);
      int eb = ((data[p + 2] & 0xff) << 8) | (data[p + 3] & 0xff);
      if (ea != dlonA || eb != dlatA) continue;
      if (readInt(data, p + 4) == dlonB && readInt(data, p + 8) == dlatB) return data[p + 12];
    }
    return -1;
  }

  private static int readInt(byte[] b, int p) {
    return ((b[p] & 0xff) << 24) | ((b[p + 1] & 0xff) << 16) | ((b[p + 2] & 0xff) << 8) | (b[p + 3] & 0xff);
  }

  private Tile tile(int ilon, int ilat) {
    int lonDeg = ilon / 1000000 - 180;
    int latDeg = ilat / 1000000 - 90;
    int tlon = Math.floorDiv(lonDeg, TILE_DEG) * TILE_DEG;
    int tlat = Math.floorDiv(latDeg, TILE_DEG) * TILE_DEG;
    int key = (tlon + 180) * 1000 + (tlat + 90);
    if (tiles.containsKey(key)) return tiles.get(key);
    Tile t = null;
    String name = (tlon < 0 ? "W" + (-tlon) : "E" + tlon) + "_" + (tlat < 0 ? "S" + (-tlat) : "N" + tlat) + ".curv";
    File f = new File(segmentDir, name);
    if (f.isFile()) {
      try {
        RandomAccessFile raf = new RandomAccessFile(f, "r");
        byte[] head = new byte[12];
        raf.readFully(head);
        if (head[0] == 'C' && head[1] == 'U' && head[2] == 'R' && head[3] == 'V' && head[4] == 0 && head[5] == 0) {
          t = new Tile();
          t.file = raf;
          t.divisor = ((head[10] & 0xff) << 8) | (head[11] & 0xff);
          t.cellsPerSide = TILE_DEG * t.divisor;
          int cellSize = 1000000 / t.divisor;
          t.lon0Cell = (tlon + 180) * 1000000 / cellSize;
          t.lat0Cell = (tlat + 90) * 1000000 / cellSize;
          int n = t.cellsPerSide * t.cellsPerSide + 1;
          byte[] idx = new byte[n * 4];
          raf.readFully(idx);
          t.offsets = new int[n];
          for (int i = 0; i < n; i++) t.offsets[i] = readInt(idx, i * 4);
          bytesLoaded += 12 + idx.length;
        } else {
          raf.close(); // unknown version: neutral
        }
      } catch (IOException e) {
        t = null;
      }
    }
    tiles.put(key, t);
    return t;
  }

  private byte[] cellData(Tile t, int cell) {
    byte[] d = t.cells.get(cell);
    if (d != null || t.cells.containsKey(cell)) return d;
    int from = t.offsets[cell];
    int to = t.offsets[cell + 1];
    if (to > from) {
      try {
        d = new byte[(to - from) * ENTRY_BYTES];
        long base = 12L + 4L * t.offsets.length;
        t.file.seek(base + (long) from * ENTRY_BYTES);
        t.file.readFully(d);
        cellsLoaded++;
        bytesLoaded += d.length;
      } catch (IOException e) {
        d = null;
      }
    }
    t.cells.put(cell, d);
    return d;
  }

  public String stats() {
    return "curvature lookups=" + lookups + " hits=" + hits + " variants=" + variantsBuilt + " cells=" + cellsLoaded + " bytes=" + bytesLoaded;
  }
}
