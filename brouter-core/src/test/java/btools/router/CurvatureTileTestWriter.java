package btools.router;

import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Writes curvature tiles (format version 1, Curvey's {@code docs/curvature-tiles.md}) for tests, independently of
 * Curvey's writer: the reader is tested against the spec, not against the other implementation.
 */
final class CurvatureTileTestWriter {
  private static final int MICRODEGREES = 1000000;
  private static final int TILE_DEGREES = 5;

  /** One link: both ends (microdegrees) and its class code. */
  static final class Link {
    final int ilonA, ilatA, ilonB, ilatB, curveClass;

    Link(int ilonA, int ilatA, int ilonB, int ilatB, int curveClass) {
      this.ilonA = ilonA;
      this.ilatA = ilatA;
      this.ilonB = ilonB;
      this.ilatB = ilatB;
      this.curveClass = curveClass;
    }
  }

  private CurvatureTileTestWriter() {
  }

  static File tileFile(File dir, int lonDeg, int latDeg) {
    String name = (lonDeg < 0 ? "W" + (-lonDeg) : "E" + lonDeg) + "_" + (latDeg < 0 ? "S" + (-latDeg) : "N" + latDeg);
    return new File(dir, name + ".curv");
  }

  /** Writes every tile any link end falls into, version 1, class table 1, divisor 32. */
  static void writeAll(File dir, List<Link> links) throws IOException {
    java.util.Set<Long> corners = new java.util.TreeSet<>();
    for (Link l : links) {
      corners.add(corner(l.ilonA, l.ilatA));
      corners.add(corner(l.ilonB, l.ilatB));
    }
    for (long c : corners) {
      int lonDeg = (int) (c >> 32);
      int latDeg = (int) c;
      write(tileFile(dir, lonDeg, latDeg), lonDeg, latDeg, 32, links, 1, 1, java.util.Collections.<String, String>emptyMap());
    }
  }

  private static long corner(int ilon, int ilat) {
    long lon = Math.floorDiv(ilon / MICRODEGREES - 180, TILE_DEGREES) * TILE_DEGREES;
    long lat = Math.floorDiv(ilat / MICRODEGREES - 90, TILE_DEGREES) * TILE_DEGREES;
    return (lon << 32) | (lat & 0xffffffffL);
  }

  static void write(File file, int lonDeg, int latDeg, int divisor, List<Link> links, int version, int classTable,
                    Map<String, String> metadata) throws IOException {
    int cellSize = MICRODEGREES / divisor;
    int side = TILE_DEGREES * divisor;
    int ilon0 = (lonDeg + 180) * MICRODEGREES;
    int ilat0 = (latDeg + 90) * MICRODEGREES;
    List<long[]> entries = new ArrayList<>(); // cell, first, second, class
    for (Link l : links) {
      if (l.curveClass == 0) continue;
      long a = point(l.ilonA, l.ilatA);
      long b = point(l.ilonB, l.ilatB);
      boolean same = l.ilonA / cellSize == l.ilonB / cellSize && l.ilatA / cellSize == l.ilatB / cellSize;
      if (same) {
        add(entries, Math.min(a, b), Math.max(a, b), l.curveClass, ilon0, ilat0, cellSize, side);
      } else {
        add(entries, a, b, l.curveClass, ilon0, ilat0, cellSize, side);
        add(entries, b, a, l.curveClass, ilon0, ilat0, cellSize, side);
      }
    }
    entries.sort((x, y) -> x[0] != y[0] ? Long.compare(x[0], y[0]) : x[1] != y[1] ? Long.compare(x[1], y[1]) : Long.compare(x[2], y[2]));
    int[] starts = new int[side * side + 1];
    ByteArrayOutputStream data = new ByteArrayOutputStream();
    int i = 0;
    for (int cell = 0; cell < side * side; cell++) {
      starts[cell] = data.size();
      int from = i;
      while (i < entries.size() && entries.get(i)[0] == cell) i++;
      if (i == from) continue;
      varint(data, i - from);
      long previous = 0;
      for (int k = from; k < i; k++) {
        long[] e = entries.get(k);
        int lon = (int) (e[1] >>> 32);
        int lat = (int) e[1];
        long offset = (long) (lon % cellSize) * cellSize + (lat % cellSize);
        varint(data, offset - previous);
        previous = offset;
        varint(data, zigzag((long) (int) (e[2] >>> 32) - lon));
        varint(data, zigzag((long) (int) e[2] - lat));
        data.write((int) e[3]);
      }
    }
    starts[side * side] = data.size();
    try (DataOutputStream out = new DataOutputStream(new FileOutputStream(file))) {
      out.writeBytes("CURV");
      out.writeShort(version);
      out.writeShort(classTable);
      out.writeShort(lonDeg);
      out.writeShort(latDeg);
      out.writeShort(divisor);
      out.writeShort(metadata.size());
      for (Map.Entry<String, String> m : metadata.entrySet()) {
        out.writeUTF(m.getKey());
        out.writeUTF(m.getValue());
      }
      for (int s : starts) out.writeInt(s);
      data.writeTo(out);
    }
  }

  private static void add(List<long[]> entries, long first, long second, int curveClass, int ilon0, int ilat0, int cellSize, int side) {
    int lon = (int) (first >>> 32);
    int lat = (int) first;
    int lonIndex = lon / cellSize - ilon0 / cellSize;
    int latIndex = lat / cellSize - ilat0 / cellSize;
    if (lonIndex < 0 || latIndex < 0 || lonIndex >= side || latIndex >= side) return;
    entries.add(new long[] {(long) lonIndex * side + latIndex, first, second, curveClass});
  }

  static long point(int ilon, int ilat) {
    return ((long) ilon << 32) | (ilat & 0xffffffffL);
  }

  private static long zigzag(long v) {
    return (v << 1) ^ (v >> 63);
  }

  private static void varint(ByteArrayOutputStream out, long v) {
    while (v >= 0x80) {
      out.write((int) ((v & 0x7f) | 0x80));
      v >>>= 7;
    }
    out.write((int) v);
  }
}
