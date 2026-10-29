package btools.web;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;

/** JVM baseline for spike #259: same request as the browser build, prints wall time and heap. */
public final class JvmMain {
  private JvmMain() {
  }

  public static void main(String[] args) throws Exception {
    String lonlats = args[0];
    String profile = args[1];
    String segmentDir = args[2];
    String extra = args.length > 4 ? args[4] : "";
    int runs = Integer.parseInt(System.getProperty("runs", "3"));
    String json = null;
    for (int i = 0; i < runs; i++) {
      System.gc();
      long start = System.nanoTime();
      json = WebRouter.route(lonlats, profile, segmentDir, extra);
      long ms = (System.nanoTime() - start) / 1_000_000;
      Runtime rt = Runtime.getRuntime();
      System.out.println("jvm run " + (i + 1) + ": " + ms + " ms, heap used " + (rt.totalMemory() - rt.freeMemory()) / (1024 * 1024) + " MB");
    }
    Files.write(Paths.get(args[3]), json.getBytes(StandardCharsets.UTF_8));
  }
}
