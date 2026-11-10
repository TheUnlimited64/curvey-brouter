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
 * Curvey: optional curvature tiles next to the routing tiles ({@code E10_N50.curv} next to {@code E10_N50.rd5}).
 * While a cell is woven, a link with a record gets the pseudo-tag {@code curvey_curve_class} (lookups.dat values
 * straight, gentle, moderate, curvy, twisty) added to its way description, so the profile prices it in normal syntax.
 *
 * <p>Neutral by design: no file, a file of another format or class table version, a damaged file, a link without a
 * record, or a profile that never reads the key all leave the description unchanged, so routing is as without the
 * tiles.
 *
 * <p>Format version 1, written by Curvey's {@code :curvature} module; spec in Curvey's {@code docs/curvature-tiles.md}.
 * Big-endian: "CURV", u16 format version, u16 class table version, i16 tile lon, i16 tile lat (south-west corner,
 * degrees), u16 divisor (cells per degree), u16 metadata count, metadata as pairs of {@code writeUTF} strings, then
 * u32[side * side + 1] cell starts relative to the data (side = 5 * divisor, cell index = lonIndex * side +
 * latIndex), then the cells. A cell: varint count, then per record varint offset delta (position of the first node in
 * the cell, lonOffset * cellSize + latOffset), zigzag varint dLon and dLat of the second node, u8 class.
 *
 * <p>Key rule: seen from node A (being decoded) to node B, a link inside one cell is stored once as (min(A, B),
 * max(A, B)), comparing {@code ilon << 32 | ilat}; a link across cells is stored in A's cell as (A, B) and in B's cell
 * as (B, A).
 *
 * <p>Not thread-safe: one instance per {@link RoutingEngine}. Reads with {@link RandomAccessFile} only, like the
 * routing tiles, so it also runs in the browser build.
 */
public final class CurvatureTiles implements LinkTagHook {
  /** The lookups.dat key the classes are exposed as. */
  public static final String LOOKUP_KEY = "curvey_curve_class";
  /** Format version this reader understands. */
  public static final int FORMAT_VERSION = 1;
  /** Class table version this reader understands (Curvey's {@code CurveClass.TABLE_VERSION}). */
  public static final int CLASS_TABLE_VERSION = 1;
  /** Highest class code (twisty). */
  public static final int MAX_CLASS = 4;

  private static final int TILE_DEGREES = 5;
  private static final int MICRODEGREES = 1000000;
  private static final int FIXED_HEADER_BYTES = 16;
  /** lookups.dat value index of class 0: 0 = empty, 1 = unknown, then the values in file order. */
  private static final int VALUE_OFFSET = 2;
  private static final byte[] MAGIC = {'C', 'U', 'R', 'V'};

  private final File segmentDir;
  private final BExpressionContextWay ctx;
  private final int keyIndex;
  private final Map<Integer, Tile> tiles = new HashMap<>();
  private final IdentityHashMap<byte[], byte[][]> variants = new IdentityHashMap<>();

  /** Counters for logs and tests. */
  public long lookups, hits, variantsBuilt, cellsLoaded, bytesRead;

  /** One opened tile; {@link #MISSING} stands for "no usable file". */
  private static final class Tile {
    String problem;
    RandomAccessFile file;
    int divisor;
    int cellSize;
    int side;
    int ilon0;
    int ilat0;
    long dataStart;
    int[] starts;
    Cell[] cells;
  }

  /** Records of one cell, sorted by (first, second) as packed points. */
  private static final class Cell {
    static final Cell EMPTY = new Cell(new long[0], new long[0], new byte[0]);
    final long[] first;
    final long[] second;
    final byte[] curveClass;

    Cell(long[] first, long[] second, byte[] curveClass) {
      this.first = first;
      this.second = second;
      this.curveClass = curveClass;
    }
  }

  private CurvatureTiles(File segmentDir, BExpressionContextWay ctx, int keyIndex) {
    this.segmentDir = segmentDir;
    this.ctx = ctx;
    this.keyIndex = keyIndex;
  }

  /**
   * A hook for {@code ctx}'s profile, or null when lookups.dat has no {@link #LOOKUP_KEY} or the profile never reads
   * it (then nothing is looked up at all).
   */
  public static CurvatureTiles create(File segmentDir, BExpressionContextWay ctx) {
    if (ctx == null || segmentDir == null) return null;
    int index = ctx.getLookupKey(LOOKUP_KEY);
    if (index < 0 || !ctx.isLookupIdxUsed(index)) return null;
    return new CurvatureTiles(segmentDir, ctx, index);
  }

  @Override
  public byte[] tagLink(int ilonA, int ilatA, int ilonB, int ilatB, byte[] description) {
    lookups++;
    int curveClass = classOf(ilonA, ilatA, ilonB, ilatB);
    if (curveClass <= 0) return description;
    hits++;
    byte[][] byClass = variants.get(description);
    if (byClass == null) {
      byClass = new byte[MAX_CLASS + 1][];
      variants.put(description, byClass);
    }
    byte[] variant = byClass[curveClass];
    if (variant == null) {
      int[] lookupData = ctx.createNewLookupData();
      ctx.decode(lookupData, false, description);
      lookupData[keyIndex] = VALUE_OFFSET + curveClass;
      byte[] encoded = ctx.encode(lookupData);
      variant = ctx.unify(encoded, 0, encoded.length);
      byClass[curveClass] = variant;
      variantsBuilt++;
    }
    return variant;
  }

  /** Class code (1..4) of the link from A to B, or 0 when there is no record (straight or no data). */
  public int classOf(int ilonA, int ilatA, int ilonB, int ilatB) {
    Tile tile = tileOf(ilonA, ilatA);
    if (tile == null) return 0;
    int lonIndex = ilonA / tile.cellSize - tile.ilon0 / tile.cellSize;
    int latIndex = ilatA / tile.cellSize - tile.ilat0 / tile.cellSize;
    if (lonIndex < 0 || latIndex < 0 || lonIndex >= tile.side || latIndex >= tile.side) return 0;
    long a = point(ilonA, ilatA);
    long b = point(ilonB, ilatB);
    long first = a;
    long second = b;
    boolean sameCell = ilonA / tile.cellSize == ilonB / tile.cellSize && ilatA / tile.cellSize == ilatB / tile.cellSize;
    if (sameCell && b < a) {
      first = b;
      second = a;
    }
    Cell cell = cell(tile, lonIndex * tile.side + latIndex, lonIndex, latIndex);
    int i = find(cell, first, second);
    return i < 0 ? 0 : cell.curveClass[i];
  }

  /** Why the tile holding the point is not used, or null when it is (or was not opened yet). For logs and tests. */
  public String problemAt(int ilon, int ilat) {
    Integer key = tileKey(ilon, ilat);
    Tile tile = tiles.get(key);
    return tile == null ? null : tile.problem;
  }

  /** Closes the opened files. */
  public void close() {
    for (Tile tile : tiles.values()) {
      if (tile.file == null) continue;
      try {
        tile.file.close();
      } catch (IOException e) {
        // ignore
      }
      tile.file = null;
    }
  }

  public String stats() {
    return "curvature lookups=" + lookups + " hits=" + hits + " variants=" + variantsBuilt + " cells=" + cellsLoaded
      + " bytes=" + bytesRead;
  }

  private static long point(int ilon, int ilat) {
    return ((long) ilon << 32) | (ilat & 0xffffffffL);
  }

  private static int find(Cell cell, long first, long second) {
    int low = 0;
    int high = cell.first.length - 1;
    while (low <= high) {
      int mid = (low + high) >>> 1;
      int cmp = cell.first[mid] != first ? Long.compare(cell.first[mid], first) : Long.compare(cell.second[mid], second);
      if (cmp < 0) {
        low = mid + 1;
      } else if (cmp > 0) {
        high = mid - 1;
      } else {
        return mid;
      }
    }
    return -1;
  }

  private static Integer tileKey(int ilon, int ilat) {
    int lon = Math.floorDiv(ilon / MICRODEGREES - 180, TILE_DEGREES) * TILE_DEGREES;
    int lat = Math.floorDiv(ilat / MICRODEGREES - 90, TILE_DEGREES) * TILE_DEGREES;
    return (lon + 180) * 1000 + (lat + 90);
  }

  /** The usable tile holding the point, or null. Opens it on first use. */
  private Tile tileOf(int ilon, int ilat) {
    Integer key = tileKey(ilon, ilat);
    Tile tile = tiles.get(key);
    if (tile == null) {
      tile = open(key / 1000 - 180, key % 1000 - 90);
      tiles.put(key, tile);
    }
    return tile.problem == null ? tile : null;
  }

  private Tile open(int lonDeg, int latDeg) {
    Tile tile = new Tile();
    String name = (lonDeg < 0 ? "W" + (-lonDeg) : "E" + lonDeg) + "_" + (latDeg < 0 ? "S" + (-latDeg) : "N" + latDeg) + ".curv";
    File file = new File(segmentDir, name);
    if (!file.isFile()) {
      tile.problem = "missing";
      return tile;
    }
    try {
      RandomAccessFile in = new RandomAccessFile(file, "r");
      tile.file = in;
      byte[] header = new byte[FIXED_HEADER_BYTES];
      in.readFully(header);
      for (int i = 0; i < MAGIC.length; i++) {
        if (header[i] != MAGIC[i]) return fail(tile, "not a curvature tile");
      }
      int version = u16(header, 4);
      if (version != FORMAT_VERSION) return fail(tile, "format version " + version);
      int classTable = u16(header, 6);
      if (classTable != CLASS_TABLE_VERSION) return fail(tile, "class table version " + classTable);
      if ((short) u16(header, 8) != lonDeg || (short) u16(header, 10) != latDeg) return fail(tile, "tile corner does not match the name");
      tile.divisor = u16(header, 12);
      if (tile.divisor <= 0 || MICRODEGREES % tile.divisor != 0) return fail(tile, "divisor " + tile.divisor);
      int metadata = u16(header, 14);
      for (int i = 0; i < 2 * metadata; i++) {
        int length = in.readUnsignedShort();
        in.seek(in.getFilePointer() + length);
      }
      tile.cellSize = MICRODEGREES / tile.divisor;
      tile.side = TILE_DEGREES * tile.divisor;
      tile.ilon0 = (lonDeg + 180) * MICRODEGREES;
      tile.ilat0 = (latDeg + 90) * MICRODEGREES;
      int count = tile.side * tile.side + 1;
      byte[] table = new byte[4 * count];
      in.readFully(table);
      tile.starts = new int[count];
      for (int i = 0; i < count; i++) {
        tile.starts[i] = i32(table, 4 * i);
        if (tile.starts[i] < 0 || (i > 0 && tile.starts[i] < tile.starts[i - 1])) return fail(tile, "cell table not ascending");
      }
      tile.dataStart = in.getFilePointer();
      if (tile.dataStart + tile.starts[count - 1] != in.length()) return fail(tile, "size does not match the cell table");
      tile.cells = new Cell[count - 1];
      bytesRead += tile.dataStart;
      return tile;
    } catch (IOException e) {
      return fail(tile, "read error: " + e.getMessage());
    }
  }

  private Tile fail(Tile tile, String problem) {
    tile.problem = problem;
    if (tile.file != null) {
      try {
        tile.file.close();
      } catch (IOException e) {
        // ignore
      }
      tile.file = null;
    }
    return tile;
  }

  /** The decoded cell; a damaged cell counts as empty. */
  private Cell cell(Tile tile, int index, int lonIndex, int latIndex) {
    Cell cell = tile.cells[index];
    if (cell != null) return cell;
    int from = tile.starts[index];
    int to = tile.starts[index + 1];
    cell = Cell.EMPTY;
    if (to > from && tile.file != null) {
      try {
        byte[] data = new byte[to - from];
        tile.file.seek(tile.dataStart + from);
        tile.file.readFully(data);
        bytesRead += data.length;
        cell = decodeCell(data, tile.ilon0 + lonIndex * tile.cellSize, tile.ilat0 + latIndex * tile.cellSize, tile.cellSize);
        cellsLoaded++;
      } catch (IOException | RuntimeException e) {
        cell = Cell.EMPTY;
      }
    }
    tile.cells[index] = cell;
    return cell;
  }

  private static Cell decodeCell(byte[] data, int cellLon0, int cellLat0, int cellSize) throws IOException {
    int[] position = {0};
    int count = (int) varint(data, position);
    if (count < 0 || count > data.length) throw new IOException("bad record count");
    long[] first = new long[count];
    long[] second = new long[count];
    byte[] curveClass = new byte[count];
    long offset = 0;
    for (int i = 0; i < count; i++) {
      offset += varint(data, position);
      int lon = cellLon0 + (int) (offset / cellSize);
      int lat = cellLat0 + (int) (offset % cellSize);
      long dLon = unzigzag(varint(data, position));
      long dLat = unzigzag(varint(data, position));
      if (position[0] >= data.length) throw new IOException("truncated cell");
      int code = data[position[0]++];
      if (code < 1 || code > MAX_CLASS) throw new IOException("class " + code);
      first[i] = point(lon, lat);
      second[i] = point((int) (lon + dLon), (int) (lat + dLat));
      curveClass[i] = (byte) code;
      if (i > 0 && (first[i] < first[i - 1] || (first[i] == first[i - 1] && second[i] <= second[i - 1]))) {
        throw new IOException("records not sorted");
      }
    }
    if (position[0] != data.length) throw new IOException("trailing bytes");
    return new Cell(first, second, curveClass);
  }

  private static long varint(byte[] data, int[] position) throws IOException {
    long result = 0;
    for (int shift = 0; shift < 64; shift += 7) {
      if (position[0] >= data.length) throw new IOException("truncated varint");
      int b = data[position[0]++] & 0xff;
      result |= (long) (b & 0x7f) << shift;
      if (b < 0x80) return result;
    }
    throw new IOException("varint too long");
  }

  private static long unzigzag(long value) {
    return (value >>> 1) ^ -(value & 1);
  }

  private static int u16(byte[] b, int p) {
    return ((b[p] & 0xff) << 8) | (b[p + 1] & 0xff);
  }

  private static int i32(byte[] b, int p) {
    return ((b[p] & 0xff) << 24) | ((b[p + 1] & 0xff) << 16) | ((b[p + 2] & 0xff) << 8) | (b[p + 3] & 0xff);
  }
}
